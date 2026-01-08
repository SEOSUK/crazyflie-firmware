// su_wrench_dob_observer.c

#include "su_wrench_dob_observer.h"
#include "su_params.h"
#include "platform_defaults.h"   // THRUST_MAX, THRUST2TORQUE, ARM_LENGTH
#include "log.h"
#include "param.h"
#include "debug.h"
#include "config.h"
#include "pm.h"                  // pmGetBatteryVoltage
#include <math.h>
#include <float.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

// ==============================
// 내부 상태
// ==============================

// 입력 추력 (body/world)
static float su_body_input_Force[3];
static float su_world_input_Force[3];
static float su_world_input_Force_scaled[3];

// 배터리 전압 / LPF
static float su_vbat_filt = 4.0f;
static float su_vbat_log  = 0.0f;

// dt 모니터링
static TickType_t su_prev_tick;
static float su_dt_filt = 0.004f;  // [s]
static float su_dt_mon  = 0.004f;  // [s]
static float su_dt_raw  = 0.004f;  // [s]

// dt 고정 사용 여부
static uint8_t su_dt_fixed_enable = 1;       // 1: 고정 dt
static float   su_dt_fixed_value  = 0.004f;  // [s] 250 Hz

// DOB 2차 필터 상태: 각 축별 (x1_v, x2_v), (x1_u, x2_u)
static float su_dob_x1_v[3] = {0.0f, 0.0f, 0.0f};
static float su_dob_x2_v[3] = {0.0f, 0.0f, 0.0f};
static float su_dob_x1_u[3] = {0.0f, 0.0f, 0.0f};
static float su_dob_x2_u[3] = {0.0f, 0.0f, 0.0f};

// 외란 추정 출력 (World, 병진 3축만 사용)
// *** su_dob_wext_hat_raw6 = "드론이 환경에 가하는 힘" (drone → env) ***
static float su_dob_wext_hat_raw6[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// 간단 1차 LPF
static inline float lpf1(float y_prev, float x, float alpha)
{
  return y_prev + alpha * (x - y_prev);
}

// ==============================
// DOB 한 축 업데이트
//  - v_in: 속도 (m/s, World)
//  - u_in: 내부 힘 + mg (N, World)
//  - dt:  샘플링 시간
//  출력: F_ext_hat (N, World)
// ==============================
static inline float su_dob_axis_update(int axis, float v_in, float u_in, float dt)
{
  const float wn   = su_dob_wn;    // [rad/s], su_params에서 정의
  const float zeta = su_dob_zeta;  // 감쇠비 (버터워스면 ~0.707)

  // ---- 속도 경로: H_v(s) = m s Q(s)
  float x1v = su_dob_x1_v[axis];
  float x2v = su_dob_x2_v[axis];

  float x1v_dot = x2v;
  float x2v_dot = -wn * wn * x1v - 2.0f * zeta * wn * x2v + v_in;

  x1v += dt * x1v_dot;
  x2v += dt * x2v_dot;

  su_dob_x1_v[axis] = x1v;
  su_dob_x2_v[axis] = x2v;

  // y_v = m * wn^2 * x2v  ≈ m s Q(s) v
  float y_v = su_mass * wn * wn * x2v;

  // ---- 입력 경로: H_u(s) = Q(s)
  float x1u = su_dob_x1_u[axis];
  float x2u = su_dob_x2_u[axis];

  float x1u_dot = x2u;
  float x2u_dot = -wn * wn * x1u - 2.0f * zeta * wn * x2u + u_in;

  x1u += dt * x1u_dot;
  x2u += dt * x2u_dot;

  su_dob_x1_u[axis] = x1u;
  su_dob_x2_u[axis] = x2u;

  // y_u = wn^2 * x1u ≈ Q(s) u
  float y_u = wn * wn * x1u;

  // 최종 외란 추정: Q(msv - u)
  float Fext_hat = y_v - y_u;
  return Fext_hat;
}

// ==============================
// 초기화
// ==============================
void suWrenchObserverDOBInit(void)
{
  for (int i = 0; i < 3; ++i) {
    su_body_input_Force[i]         = 0.0f;
    su_world_input_Force[i]        = 0.0f;
    su_world_input_Force_scaled[i] = 0.0f;

    su_dob_x1_v[i] = 0.0f;
    su_dob_x2_v[i] = 0.0f;
    su_dob_x1_u[i] = 0.0f;
    su_dob_x2_u[i] = 0.0f;
  }

  // 전압 초기화
  float v0 = pmGetBatteryVoltage();
  if (isfinite(v0) && v0 > 0.5f) {
    su_vbat_filt = v0;
  }
  su_vbat_log = su_vbat_filt;

  // dt 초기값
  su_prev_tick = xTaskGetTickCount();
  su_dt_filt   = su_dt_fixed_value;
  su_dt_mon    = su_dt_filt;
  su_dt_raw    = su_dt_filt;

  for (int i = 0; i < 6; ++i) {
    su_dob_wext_hat_raw6[i] = 0.0f;
  }

  DEBUG_PRINT("SU Wrench DOB observer initialized (use external velocity)\n");
}

// ==============================
// 업데이트 (250 Hz 정도)
//  vW: World-frame velocity [m/s], su_vel_from_pos 에서 계산된 값
// ==============================
void suWrenchObserverDOBUpdate(const state_t *state,
                               const motors_thrust_pwm_t *motorPwm,
                               const Axis3f *gyro_deg_s,
                               const float vW[3])
{
  (void)gyro_deg_s; // 병진 DOB에서는 사용 안 함

  // ---- 0) dt 측정 + LPF
  TickType_t now      = xTaskGetTickCount();
  TickType_t dt_ticks = now - su_prev_tick;
  su_prev_tick        = now;

  float dt_meas = (float)dt_ticks * ((float)portTICK_PERIOD_MS * 1e-3f);
  if (isfinite(dt_meas) && dt_meas > 0.0f && dt_meas < 0.05f) {
    su_dt_filt = lpf1(su_dt_filt, dt_meas, su_dt_alpha);
    if (su_dt_filt < 1e-4f) su_dt_filt = 1e-4f;
    if (su_dt_filt > 5e-2f) su_dt_filt = 5e-2f;
    su_dt_raw = dt_meas;
  }
  su_dt_mon = su_dt_filt;

  const float dt = su_dt_fixed_enable ? su_dt_fixed_value : su_dt_filt;

  // ---- 1) 배터리 전압 측정 + LPF
  float v_meas = pmGetBatteryVoltage();
  su_vbat_filt += su_vbat_alpha * (v_meas - su_vbat_filt);
  su_vbat_log = su_vbat_filt;

  // ---- 2) PWM -> 추력 [N], 전압 모델 반영
  const float pwm2N = (THRUST_MAX / (float)UINT16_MAX);

  float voltage_model = su_voltage_model_a * su_vbat_filt + su_voltage_model_b;
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

  // Body-frame Force (z축만)
  const float Fz       = f1 + f2 + f3 + f4;
  const float Fz_scale = f1_scale + f2_scale + f3_scale + f4_scale;

  su_body_input_Force[0] = 0.0f;
  su_body_input_Force[1] = 0.0f;
  su_body_input_Force[2] = Fz;

  // ---- 3) Body -> World (쿼터니언)
  {
    float qx = state->attitudeQuaternion.x;
    float qy = state->attitudeQuaternion.y;
    float qz = state->attitudeQuaternion.z;
    float qw = state->attitudeQuaternion.w;

    const float n = sqrtf(qx*qx + qy*qy + qz*qz + qw*qw);
    if (n > 1e-6f) { qx/=n; qy/=n; qz/=n; qw/=n; }

    const float xx = qx*qx;
    const float yy = qy*qy;
    const float xz = qx*qz;
    const float yz = qy*qz;
    const float xw = qx*qw;
    const float yw = qy*qw;

    const float R13 = 2.0f*(xz + yw);
    const float R23 = 2.0f*(yz - xw);
    const float R33 = 1.0f - 2.0f*(xx + yy);

    su_world_input_Force[0]        = R13 * Fz;
    su_world_input_Force[1]        = R23 * Fz;
    su_world_input_Force[2]        = R33 * Fz;

    su_world_input_Force_scaled[0] = R13 * Fz_scale;
    su_world_input_Force_scaled[1] = R23 * Fz_scale;
    su_world_input_Force_scaled[2] = R33 * Fz_scale;
  }

  // ---- 4) 외부에서 전달 받은 World velocity 사용
  float vWx = (vW != NULL) ? vW[0] : 0.0f;
  float vWy = (vW != NULL) ? vW[1] : 0.0f;
  float vWz = (vW != NULL) ? vW[2] : 0.0f;

  if (!isfinite(vWx)) vWx = 0.0f;
  if (!isfinite(vWy)) vWy = 0.0f;
  if (!isfinite(vWz)) vWz = 0.0f;

  // ---- 5) 내부 힘 + mg (World) → u_i
  const float GRAV = 9.80665f;
  float uW[3];

  uW[0] = su_world_input_Force_scaled[0];                   // x: 내부 힘만
  uW[1] = su_world_input_Force_scaled[1];                   // y: 내부 힘만
  uW[2] = su_world_input_Force_scaled[2] - su_mass * GRAV;  // z: 내부힘 + m g

  // ---- 6) DOB 축별 업데이트
  float vW_local[3] = { vWx, vWy, vWz };
  for (int i = 0; i < 3; ++i) {
    float Fext_hat = su_dob_axis_update(i, vW_local[i], uW[i], dt);
    // Fext_hat = env → drone (외란)
    // 우리가 쓰고 싶은 건 drone → env 이므로 부호 반전
    su_dob_wext_hat_raw6[i] = -Fext_hat;  // X,Y,Z, 드론이 환경에 가하는 힘
  }

  // 토크 성분은 0 (지금은 병진만 고려)
  su_dob_wext_hat_raw6[3] = 0.0f;
  su_dob_wext_hat_raw6[4] = 0.0f;
  su_dob_wext_hat_raw6[5] = 0.0f;
}


// ==============================
// 외부에서 DOB 힘 읽기 (World, drone → env)
// ==============================
void suWrenchObserverDOBGetWorldForce(float outF[3])
{
  if (!outF) {
    return;
  }

  // su_dob_wext_hat_raw6[0..2] = 드론이 환경에 가하는 힘 (drone → env, World)
  outF[0] = su_dob_wext_hat_raw6[0];
  outF[1] = su_dob_wext_hat_raw6[1];
  outF[2] = su_dob_wext_hat_raw6[2];
}


// ==============================
// 로그 그룹
// ==============================
// LOG_GROUP_START(suWrenchDOB)

// LOG_ADD(LOG_FLOAT, suWFx_scaled, &su_world_input_Force_scaled[0])
// LOG_ADD(LOG_FLOAT, suWFy_scaled, &su_world_input_Force_scaled[1])
// LOG_ADD(LOG_FLOAT, suWFz_scaled, &su_world_input_Force_scaled[2])

// LOG_ADD(LOG_FLOAT, suV,     &su_vbat_log)
// LOG_ADD(LOG_FLOAT, suDt,    &su_dt_mon)
// LOG_ADD(LOG_FLOAT, suDtraw, &su_dt_raw)

// LOG_ADD(LOG_FLOAT, suFextX, &su_dob_wext_hat_raw6[0])
// LOG_ADD(LOG_FLOAT, suFextY, &su_dob_wext_hat_raw6[1])
// LOG_ADD(LOG_FLOAT, suFextZ, &su_dob_wext_hat_raw6[2])

// LOG_GROUP_STOP(suWrenchDOB)
