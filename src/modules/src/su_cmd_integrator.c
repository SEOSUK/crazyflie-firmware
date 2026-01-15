// su_cmd_integrator.c

#include "su_cmd_integrator.h"
#include "param.h"
#include "su_params.h"
#include <stdbool.h>
#include "log.h"

// ===========================
// 내부 상태
// ===========================
static float su_int_pos_x = 0.0f;
static float su_int_pos_y = 0.0f;
static float su_int_pos_z = 0.0f;
static float su_int_yaw   = 0.0f;

// vel 모드 ↔ pos 모드 전환 감지용
static bool su_vel_mode_prev    = false;
static bool su_int_initialized  = false;

// ★ vel -> pos 전환 이후, pos 명령을 "변위 명령"으로 쓰기 위한 기준점
static float su_pos_base_x = 0.0f;
static float su_pos_base_y = 0.0f;
static float su_pos_base_z = 0.0f;
static float su_yaw_base   = 0.0f;
static bool  su_pos_base_valid = false;

// 파라미터: 0=position mode, 1=velocity mode
static float su_cmd_use_vel_mode = 1.0f;

// ★ 힘 커맨드 파라미터 (예: x축 force 명령)
static float su_cmd_fx = 0.0f;

// ★ 어드미턴스 파라미터
static float su_adm_M = 1.0f;
static float su_adm_D = 0.0f;
static float su_adm_K = 0.0f;

// ★ 어드미턴스 내부 상태 (x 방향만 사용)
//   v_adm = K * ∫(F_des - F_meas) dt  +  D * (F_des - F_meas)  -  M * F_dot_filtered
//   - su_adm_int_Ferr_x : ∫(F_des - F_meas) dt  (force error 적분)
//   - su_adm_Fmeas_prev_x : F_meas 이전값 (수치미분용)
//   - su_adm_Fdot_lpf_x   : 1차 LPF 통과된 F_dot
static float su_adm_int_Ferr_x    = 0.0f;
static float su_adm_Fmeas_prev_x  = 0.0f;
static float su_adm_Fdot_lpf_x    = 0.0f;
static bool  su_adm_initialized   = false;

// ===========================
// PARAM GROUP
// ===========================
PARAM_GROUP_START(su_cmd)
  PARAM_ADD(PARAM_FLOAT, use_vel_mode, &su_cmd_use_vel_mode)
  PARAM_ADD(PARAM_FLOAT, cmd_fx,       &su_cmd_fx)
  PARAM_ADD(PARAM_FLOAT, adm_M,        &su_adm_M)
  PARAM_ADD(PARAM_FLOAT, adm_D,        &su_adm_D)
  PARAM_ADD(PARAM_FLOAT, adm_K,        &su_adm_K)
PARAM_GROUP_STOP(su_cmd)

void suCmdIntegratorInit(void)
{
  su_int_pos_x = 0.0f;
  su_int_pos_y = 0.0f;
  su_int_pos_z = 0.0f;
  su_int_yaw   = 0.0f;

  su_pos_base_x = 0.0f;
  su_pos_base_y = 0.0f;
  su_pos_base_z = 0.0f;
  su_yaw_base   = 0.0f;
  su_pos_base_valid = false;

  su_vel_mode_prev   = (su_cmd_use_vel_mode > 0.5f);
  su_int_initialized = false;

  // ★ 어드미턴스 내부 상태 초기화
  su_adm_int_Ferr_x   = 0.0f;
  su_adm_Fmeas_prev_x = 0.0f;
  su_adm_Fdot_lpf_x   = 0.0f;
  su_adm_initialized  = false;
}


// dt        : 호출 주기 (예: 1/1000)
// sp_in     : 원본 setpoint
// state     : 현재 상태
// f_ext_world : [Fx, Fy, Fz] (World frame force estimate, NULL 허용)
// sp_out    : 수정된 setpoint (항상 absolute position/yaw 모드로 출력)
void suCmdIntegratorUpdate(float dt,
                           const setpoint_t* sp_in,
                           const state_t* state,
                           const float* f_ext_world,
                           setpoint_t* sp_out)
{
  // 기본적으로 sp_out에 sp_in 복사해두고, 그 위에 position/yaw만 덮어쓴다
  *sp_out = *sp_in;

  const bool vel_mode_on = (su_cmd_use_vel_mode > 0.5f);

  // 0) 첫 호출 시: 현재 상태 기준으로 내부 적분 상태 초기화
  if (!su_int_initialized) {
    su_int_pos_x = state->position.x;
    su_int_pos_y = state->position.y;
    su_int_pos_z = state->position.z;
    su_int_yaw   = state->attitude.yaw;  // [deg]
    su_int_initialized = true;

    // ★ 어드미턴스 쪽도 함께 초기화
    su_adm_int_Ferr_x   = 0.0f;
    su_adm_Fmeas_prev_x = 0.0f;
    su_adm_Fdot_lpf_x   = 0.0f;
    su_adm_initialized  = false;
  }

  // 1) 모드 전환 감지
  if (vel_mode_on && !su_vel_mode_prev) {
    // pos -> vel 전환 순간: su_int_* 유지, pos 기준점 invalidate
    su_pos_base_valid = false;

    // ★ 어드미턴스 상태도 리셋
    su_adm_int_Ferr_x   = 0.0f;
    su_adm_Fmeas_prev_x = 0.0f;
    su_adm_Fdot_lpf_x   = 0.0f;
    su_adm_initialized  = false;

  } else if (!vel_mode_on && su_vel_mode_prev) {
    // vel -> pos 전환 순간: 내부 적분 상태를 기준점으로 저장
    su_pos_base_x = su_int_pos_x;
    su_pos_base_y = su_int_pos_y;
    su_pos_base_z = su_int_pos_z;
    su_yaw_base   = su_int_yaw;
    su_pos_base_valid = true;

    // ★ pos 모드 들어갈 때도 어드미턴스 상태 정리
    su_adm_int_Ferr_x   = 0.0f;
    su_adm_Fmeas_prev_x = 0.0f;
    su_adm_Fdot_lpf_x   = 0.0f;
    su_adm_initialized  = false;
  }
  su_vel_mode_prev = vel_mode_on;

  // 2) 모드별 처리
  if (!vel_mode_on) {
    // ==========================
    // Position mode
    // ==========================
    if (su_pos_base_valid) {
      // vel -> pos 이후: position = 기준점 기준 변위 [m]
      su_int_pos_x = su_pos_base_x + sp_in->position.x;
      su_int_pos_y = su_pos_base_y + sp_in->position.y;
      su_int_pos_z = su_pos_base_z + sp_in->position.z;
      su_int_yaw   = su_yaw_base   + sp_in->attitude.yaw;
    } else {
      // 초기 pos 모드: 절대 위치 명령
      su_int_pos_x = sp_in->position.x;
      su_int_pos_y = sp_in->position.y;
      su_int_pos_z = sp_in->position.z;
      su_int_yaw   = sp_in->attitude.yaw;
    }

  } else {
    // ==========================
    // Velocity mode
    // ==========================
    float vx_cmd   = sp_in->position.x;
    float vy_cmd   = sp_in->position.y;
    float vz_cmd   = sp_in->position.z;
    float vyaw_cmd = sp_in->attitude.yaw;   // [deg/s]

    // ----- ★ Admittance: x축 힘 기반 v_adm_x 생성 -----
    //
    //   v_adm_x = K * ∫(F_des - F_meas) dt
    //           + D * (F_des - F_meas)
    //           - M * F_dot_filtered
    //
    if (f_ext_world && dt > 0.0f) {
      const float F_meas_x = f_ext_world[0];   // World frame Fx_hat
      const float F_des_x  = su_cmd_fx;        // 원하는 힘 (command force)

      const float F_err_x  = F_des_x - F_meas_x;

      // 첫 초기화: 이전값 동기화
      if (!su_adm_initialized) {
        su_adm_Fmeas_prev_x = F_meas_x;
        su_adm_Fdot_lpf_x   = 0.0f;
        su_adm_int_Ferr_x   = 0.0f;
        su_adm_initialized  = true;
      }

      // 1) F_dot (수치미분)
      const float Fdot_raw_x = (F_meas_x - su_adm_Fmeas_prev_x) / dt;
      su_adm_Fmeas_prev_x = F_meas_x;

      // 2) 1차 LPF (cutoff = 2 Hz)
      //
      //    연속시간: y_dot = w_c (x - y),  w_c = 2π f_c
      //    이산:     y(k+1) = y(k) + dt * w_c * (x(k) - y(k))
      const float fc   = 2.0f;                          // [Hz]
      const float wc   = 2.0f * 3.14159265359f * fc;    // [rad/s]
      su_adm_Fdot_lpf_x += dt * wc * (Fdot_raw_x - su_adm_Fdot_lpf_x);

      // 3) force error 적분
      su_adm_int_Ferr_x += F_err_x * dt;

      // // 4) v_adm 계산
      // const float v_adm_x =
      //   su_adm_K * su_adm_int_Ferr_x +
      //   su_adm_D * F_err_x -
      //   su_adm_M * su_adm_Fdot_lpf_x;

      // x 방향 속도 명령에 admittance 속도 더하기
      // vx_cmd += v_adm_x;
    }

    // ----- 기존 속도 적분 -----
    su_int_pos_x += vx_cmd   * dt;
    su_int_pos_y += vy_cmd   * dt;
    su_int_pos_z += vz_cmd   * dt;
    su_int_yaw   += vyaw_cmd * dt;

    // 필요하면 yaw wrap 처리 가능
    // if (su_int_yaw > 180.0f)  su_int_yaw -= 360.0f;
    // if (su_int_yaw < -180.0f) su_int_yaw += 360.0f;
  }

  // 3) 최종 setpoint 출력 (항상 absolute pos 모드)
  sp_out->position.x   = su_int_pos_x;
  sp_out->position.y   = su_int_pos_y;
  sp_out->position.z   = su_int_pos_z;
  sp_out->attitude.yaw = su_int_yaw;

  sp_out->mode.x   = modeAbs;
  sp_out->mode.y   = modeAbs;
  sp_out->mode.z   = modeAbs;
  sp_out->mode.yaw = modeAbs;
}

// ===========================
// LOG GROUPS
// ===========================
LOG_GROUP_START(su_cmd_dbg)
LOG_ADD(LOG_FLOAT, use_vel_mode, &su_cmd_use_vel_mode)
LOG_ADD(LOG_FLOAT, cmd_fx,       &su_cmd_fx)
// LOG_ADD(LOG_FLOAT, adm_M,        &su_adm_M)
// LOG_ADD(LOG_FLOAT, adm_D,        &su_adm_D)
LOG_ADD(LOG_FLOAT, adm_K,        &su_adm_K)
LOG_GROUP_STOP(su_cmd_dbg)

LOG_GROUP_START(su_cmd_sp)
LOG_ADD(LOG_FLOAT, x_int,   &su_int_pos_x)
LOG_ADD(LOG_FLOAT, y_int,   &su_int_pos_y)
LOG_ADD(LOG_FLOAT, z_int,   &su_int_pos_z)
LOG_ADD(LOG_FLOAT, yaw_int, &su_int_yaw)
LOG_GROUP_STOP(su_cmd_sp)
