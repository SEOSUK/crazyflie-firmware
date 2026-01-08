// su_wrench_observer.c

#include "su_wrench_observer.h"
#include "su_params.h"
#include "platform_defaults.h"   // THRUST_MAX, THRUST2TORQUE, ARM_LENGTH
#include "log.h"
#include "param.h"
#include "debug.h"
#include "config.h"
#include "pm.h"                  // 배터리 전압 읽기
#include <math.h>
#include <float.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

// ==============================
// SU: 내부 상태 (로그 대상)
// ==============================
static float su_body_input_Force[3];
static float su_body_input_Force_scaled[3];

static float su_world_input_Force[3];              // [Fx, Fy, Fz] (World)
static float su_world_input_Force_scaled[3];       // [Fx, Fy, Fz] (World)

static float su_body_input_Torque[3];
static float su_body_input_Torque_scaled[3];

// === MOB 추정치 (로그 대상) ===
static float su_world_F_hat_ext[3];     // [Fx, Fy, Fz] (World, env쪽 힘)
static float su_body_Tau_hat_ext[3];    // [Tx, Ty, Tz] (body, env쪽 토크)
static float r6[6];
static float wext_hat_raw6[6];

// === 6D MOB 내부상태: p̂ = [m v_W ; J ω_B] ===
static float su_p6_hat[6] = {0};        // p̂ = [px,py,pz,Lx,Ly,Lz]

// === 실제 전압 / LPF 상태 ===
static float su_vbat_filt = 4.0f;       // 초기값: 적당한 시작 전압
static float su_vbat_log  = 0.0f;       // 로깅용

// === 시간 측정 & dt LPF ===
static TickType_t su_prev_tick;
static float su_dt_filt = 0.004f;       // [s] 250 Hz 기준, 모니터링용 필터된 dt
static float su_dt_mon  = 0.004f;       // 로깅용(필터된 dt)
static float su_dt_raw  = 0.004f;       // 로깅용(생 dt)

// 제어/적분용 고정 dt
static uint8_t su_dt_fixed_enable = 1;     // 1=고정 dt 사용
static float   su_dt_fixed_value  = 0.004f; // [s] 250 Hz

// === LPF helper ===
static inline float lpf1(float y_prev, float x, float alpha) {
    return y_prev + alpha * (x - y_prev);
}

void suWrenchObserverInit(void) {

    for (int i = 0; i < 3; ++i) {
        su_body_input_Force[i]         = 0.0f;
        su_body_input_Force_scaled[i]  = 0.0f;
        su_world_input_Force[i]        = 0.0f;
        su_world_input_Force_scaled[i] = 0.0f;
        su_body_input_Torque[i]        = 0.0f;
        su_body_input_Torque_scaled[i] = 0.0f;
        su_world_F_hat_ext[i]          = 0.0f;
        su_body_Tau_hat_ext[i]         = 0.0f;
    }

    // 초기 전압 샘플로 LPF 초기화
    float v0 = pmGetBatteryVoltage();           // [V]
    if (isfinite(v0) && v0 > 0.5f) {
        su_vbat_filt = v0;
    }
    su_vbat_log = su_vbat_filt;

    // MOB 내부상태/출력 0으로
    for (int i = 0; i < 6; ++i) {
        su_p6_hat[i]     = 0.0f;
        r6[i]            = 0.0f;
        wext_hat_raw6[i] = 0.0f;
    }

    // dt LPF용 초기 타임스탬프/값
    su_prev_tick = xTaskGetTickCount();
    su_dt_filt   = su_dt_fixed_value;
    su_dt_mon    = su_dt_filt;
    su_dt_raw    = su_dt_filt;

    DEBUG_PRINT("SU Wrench observer initialized (use external velocity)\n");
}

void suWrenchObserverUpdate(const state_t *state,
                            const motors_thrust_pwm_t *motorPwm,
                            const Axis3f *gyro_deg_s,
                            const float vW[3])
{
    // ---- (-1) dt 측정 + 1차 LPF
    TickType_t now      = xTaskGetTickCount();   // 1초에 1000씩 오름.
    TickType_t dt_ticks = now - su_prev_tick;
    su_prev_tick        = now;

    // FreeRTOS tick -> seconds (모니터링용)
    float dt_meas = (float)dt_ticks * ((float)portTICK_PERIOD_MS * 1e-3f);
    if (isfinite(dt_meas) && dt_meas > 0.0f && dt_meas < 0.05f) {
        su_dt_filt = lpf1(su_dt_filt, dt_meas, su_dt_alpha);  // su_dt_alpha from su_params
        if (su_dt_filt < 1e-4f) su_dt_filt = 1e-4f;
        if (su_dt_filt > 5e-2f) su_dt_filt = 5e-2f;
        su_dt_raw = dt_meas;
    }
    su_dt_mon = su_dt_filt; // 로그용

    // ---- 0) 실제 전압 읽기 + LPF
    float v_meas = pmGetBatteryVoltage();       // [V]

    // 1차 IIR LPF
    su_vbat_filt += su_vbat_alpha * (v_meas - su_vbat_filt);  // su_vbat_alpha from su_params
    su_vbat_log = su_vbat_filt;

    // ---- 1) PWM → Force[N]  (power_distribution와 동일 스케일)
    const float pwm2N = (THRUST_MAX / (float)UINT16_MAX);

    // ★ 실제 전압을 반영한 모델
    float voltage_model = su_voltage_model_a * su_vbat_filt + su_voltage_model_b; // from su_params
    if (voltage_model < 0.0f) {
        voltage_model = 0.0f;
    }

    const float f1 = pwm2N * (float)motorPwm->motors.m1;
    const float f2 = pwm2N * (float)motorPwm->motors.m2;
    const float f3 = pwm2N * (float)motorPwm->motors.m3;
    const float f4 = pwm2N * (float)motorPwm->motors.m4;

    const float f1_scale = voltage_model * pwm2N * (float)motorPwm->motors.m1;
    const float f2_scale = voltage_model * pwm2N * (float)motorPwm->motors.m2;
    const float f3_scale = voltage_model * pwm2N * (float)motorPwm->motors.m3;
    const float f4_scale = voltage_model * pwm2N * (float)motorPwm->motors.m4;

    // ---- 2) Body-frame Force
    const float Fz = f1 + f2 + f3 + f4;
    su_body_input_Force[0] = 0.0f;
    su_body_input_Force[1] = 0.0f;
    su_body_input_Force[2] = Fz;

    const float Fz_scale = f1_scale + f2_scale + f3_scale + f4_scale;
    su_body_input_Force_scaled[0] = 0.0f;
    su_body_input_Force_scaled[1] = 0.0f;
    su_body_input_Force_scaled[2] = Fz_scale;

    // ---- 3) Body-frame Torque (정확한 역-할당)
    const float arm = 0.707106781f * ARM_LENGTH; // r / sqrt(2)

    const float Tx = arm * ((f3 + f4) - (f1 + f2));
    const float Ty = arm * ((f2 + f3) - (f1 + f4));
    const float Tz = THRUST2TORQUE * (-f1 + f2 - f3 + f4);
    su_body_input_Torque[0] = Tx;
    su_body_input_Torque[1] = Ty;
    su_body_input_Torque[2] = Tz;
    
    const float Tx_scale = arm * ((f3_scale + f4_scale) - (f1_scale + f2_scale));
    const float Ty_scale = arm * ((f2_scale + f3_scale) - (f1_scale + f4_scale));
    const float Tz_scale = THRUST2TORQUE * (-f1_scale + f2_scale - f3_scale + f4_scale);
    su_body_input_Torque_scaled[0] = Tx_scale;
    su_body_input_Torque_scaled[1] = Ty_scale;
    su_body_input_Torque_scaled[2] = Tz_scale;

    // ---- 4) Body → World (자세 쿼터니언)
    {
        float qx = state->attitudeQuaternion.x;
        float qy = state->attitudeQuaternion.y;
        float qz = state->attitudeQuaternion.z;
        float qw = state->attitudeQuaternion.w;

        const float n = sqrtf(qx*qx + qy*qy + qz*qz + qw*qw);
        if (n > 1e-6f) { qx/=n; qy/=n; qz/=n; qw/=n; }

        // 필요한 항만 계산
        const float xx = qx*qx;
        const float yy = qy*qy;
        const float xz = qx*qz;
        const float yz = qy*qz;
        const float xw = qx*qw;
        const float yw = qy*qw;

        const float R13 = 2.0f*(xz + yw);
        const float R23 = 2.0f*(yz - xw);
        const float R33 = 1.0f - 2.0f*(xx + yy);

        su_world_input_Force[0] = R13 * Fz;
        su_world_input_Force[1] = R23 * Fz;
        su_world_input_Force[2] = R33 * Fz;

        su_world_input_Force_scaled[0] = R13 * Fz_scale;
        su_world_input_Force_scaled[1] = R23 * Fz_scale;
        su_world_input_Force_scaled[2] = R33 * Fz_scale;
    }

    // =========================
    // 6D MCG-MOB (World force + Body torque)
    // p := M v,   p̂˙ = u - C(q,v) - G(q) + K*(p - p̂)
    // \hat{w}_ext = K*(p - p̂)
    // =========================

    // ---- v_W (m/s) — 외부에서 전달된 World velocity 사용
    float vx = (vW != NULL) ? vW[0] : 0.0f;
    float vy = (vW != NULL) ? vW[1] : 0.0f;
    float vz = (vW != NULL) ? vW[2] : 0.0f;

    if (!isfinite(vx)) vx = 0.0f;
    if (!isfinite(vy)) vy = 0.0f;
    if (!isfinite(vz)) vz = 0.0f;



    // ---- ω_B (rad/s)
    const float DEG2RAD = 0.017453292519943295f;
    float wx = gyro_deg_s ? (gyro_deg_s->x * DEG2RAD) : 0.0f;
    float wy = gyro_deg_s ? (gyro_deg_s->y * DEG2RAD) : 0.0f;
    float wz = gyro_deg_s ? (gyro_deg_s->z * DEG2RAD) : 0.0f;
    if (!isfinite(wx)) wx = 0.0f;
    if (!isfinite(wy)) wy = 0.0f;
    if (!isfinite(wz)) wz = 0.0f;

    // p = M v
    const float Ix = Jxx;
    const float Iy = Jyy;
    const float Iz = Jzz;  // inertias from su_params
    float p6[6] = {
        su_mass * vx,
        su_mass * vy,
        su_mass * vz,
        Ix * wx,
        Iy * wy,
        Iz * wz
    }; // mass from su_params

    // u = [F_W ; τ_B]
    float u6[6] = {
        su_world_input_Force_scaled[0],
        su_world_input_Force_scaled[1],
        su_world_input_Force_scaled[2],
        su_body_input_Torque_scaled[0],
        su_body_input_Torque_scaled[1],
        su_body_input_Torque_scaled[2]
    };

    // C(q,v) = [0; ω×(Jω)]
    const float Iw_x = Ix * wx;
    const float Iw_y = Iy * wy;
    const float Iw_z = Iz * wz;
    float cori6[6] = {
        0.0f, 0.0f, 0.0f,
        (wy*Iw_z - wz*Iw_y),
        (wz*Iw_x - wx*Iw_z),
        (wx*Iw_y - wy*Iw_x)
    };

    // G(q) = [0,0, m g, 0,0,0]
    const float GRAV = 9.80665f;
    float grav6[6] = { 0.0f, 0.0f, su_mass * GRAV, 0.0f, 0.0f, 0.0f };

    // r = p - p̂,   \hat{w}_ext = K * r
    for (int i = 0; i < 6; i++) {
        r6[i] = p6[i] - su_p6_hat[i];
    }

    wext_hat_raw6[0] = su_Kf   * r6[0];   // Kf from su_params
    wext_hat_raw6[1] = su_Kf   * r6[1];
    wext_hat_raw6[2] = su_Kf   * r6[2];
    wext_hat_raw6[3] = su_Ktau * r6[3];  // Ktau from su_params
    wext_hat_raw6[4] = su_Ktau * r6[4];
    wext_hat_raw6[5] = su_Ktau * r6[5];

    // deadzone(옵션)
    for (int i = 0; i < 3; i++) {
        float v = wext_hat_raw6[i];
        float a = su_deadzone_F;
        wext_hat_raw6[i] = (fabsf(v) <= a) ? 0.0f : (v > 0.0f ? v - a : v + a);
    }
    for (int i = 3; i < 6; i++) { 
        float v = wext_hat_raw6[i];
        float a = su_deadzone_T;
        wext_hat_raw6[i] = (fabsf(v) <= a) ? 0.0f : (v > 0.0f ? v - a : v + a);
    }

    // p_hat 적분
    float p6_hat_dot[6];
    const float dt = su_dt_fixed_enable ? su_dt_fixed_value : su_dt_filt;
    for (int i = 0; i < 6; i++) {
        p6_hat_dot[i] = u6[i] - cori6[i] - grav6[i] + wext_hat_raw6[i];
        su_p6_hat[i] += dt * p6_hat_dot[i];
    }

    // 출력 LPF
    // wext_hat_raw6 = "드론이 받은 힘/토크" (env → drone)
    // 우리가 로그/외부에 내보내고 싶은 건 "드론이 낸 힘/토크" (drone → env)
    // => F_env = - F_ext  (Newton 3rd law)
    su_world_F_hat_ext[0] = lpf1(su_world_F_hat_ext[0], -wext_hat_raw6[0], su_mob_alpha);
    su_world_F_hat_ext[1] = lpf1(su_world_F_hat_ext[1], -wext_hat_raw6[1], su_mob_alpha);
    su_world_F_hat_ext[2] = lpf1(su_world_F_hat_ext[2], -wext_hat_raw6[2], su_mob_alpha);

    su_body_Tau_hat_ext[0] = lpf1(su_body_Tau_hat_ext[0], -wext_hat_raw6[3], su_mob_alpha);
    su_body_Tau_hat_ext[1] = lpf1(su_body_Tau_hat_ext[1], -wext_hat_raw6[4], su_mob_alpha);
    su_body_Tau_hat_ext[2] = lpf1(su_body_Tau_hat_ext[2], -wext_hat_raw6[5], su_mob_alpha);
}

void suWrenchObserverGetWorldForce(float outF[3])
{
    if (!outF) {
        return;
    }
    outF[0] = su_world_F_hat_ext[0];
    outF[1] = su_world_F_hat_ext[1];
    outF[2] = su_world_F_hat_ext[2];
}

// 로그 그룹
// LOG_GROUP_START(suWrenchObs)
// LOG_ADD(LOG_FLOAT, suFx,  &su_body_input_Force[0])
// LOG_ADD(LOG_FLOAT, suFy,  &su_body_input_Force[1])
// LOG_ADD(LOG_FLOAT, suFz,  &su_body_input_Force[2])
// LOG_ADD(LOG_FLOAT, suTx,  &su_body_input_Torque[0])
// LOG_ADD(LOG_FLOAT, suTy,  &su_body_input_Torque[1])
// LOG_ADD(LOG_FLOAT, suTz,  &su_body_input_Torque[2])

// LOG_ADD(LOG_FLOAT, suWFx, &su_world_input_Force[0])
// LOG_ADD(LOG_FLOAT, suWFy, &su_world_input_Force[1])
// LOG_ADD(LOG_FLOAT, suWFz, &su_world_input_Force[2])

// LOG_ADD(LOG_FLOAT, suWFx_scaled, &su_world_input_Force_scaled[0])
// LOG_ADD(LOG_FLOAT, suWFy_scaled, &su_world_input_Force_scaled[1])
// LOG_ADD(LOG_FLOAT, suWFz_scaled, &su_world_input_Force_scaled[2])

// LOG_ADD(LOG_FLOAT, suV,   &su_vbat_log)      // 필터링 된 전압

// // dt 로깅 & MOB 출력
// LOG_ADD(LOG_FLOAT, suDt,     &su_dt_mon)     // 모니터링용 필터된 dt
// LOG_ADD(LOG_FLOAT, suDtraw,  &su_dt_raw)     // 생 dt

// LOG_ADD(LOG_FLOAT, suFextX, &su_world_F_hat_ext[0])     // World 힘 추정 (LPF)
// LOG_ADD(LOG_FLOAT, suFextY, &su_world_F_hat_ext[1])
// LOG_ADD(LOG_FLOAT, suFextZ, &su_world_F_hat_ext[2])

// LOG_ADD(LOG_FLOAT, suFextX_raw, &wext_hat_raw6[0])      // World 힘 추정 (raw)
// LOG_ADD(LOG_FLOAT, suFextY_raw, &wext_hat_raw6[1])
// LOG_ADD(LOG_FLOAT, suFextZ_raw, &wext_hat_raw6[2])

// LOG_ADD(LOG_FLOAT, suTextX, &su_body_Tau_hat_ext[0])    // Body 토크 추정
// LOG_ADD(LOG_FLOAT, suTextY, &su_body_Tau_hat_ext[1])
// LOG_ADD(LOG_FLOAT, suTextZ, &su_body_Tau_hat_ext[2])

// LOG_GROUP_STOP(suWrenchObs)
