// su_vel_from_pos.c

#include "su_vel_from_pos.h"
#include <math.h>
#include <stdbool.h>
#include "log.h"

// ==============================
// 내부 상태
// ==============================

// 이전 위치 (World)
static float su_pos_prev[3]     = {0.0f, 0.0f, 0.0f};
// LPF 걸린 속도 (World) -> 여기만 로그에 노출
static float su_vel_lpf[3]      = {0.0f, 0.0f, 0.0f};
static bool  su_vel_initialized = false;

// 간단 1차 LPF
static inline float lpf1(float y_prev, float x, float alpha)
{
  return y_prev + alpha * (x - y_prev);
}

// ==============================
// 초기화
// ==============================
void suVelFromPosInit(void)
{
  for (int i = 0; i < 3; ++i) {
    su_pos_prev[i] = 0.0f;
    su_vel_lpf[i]  = 0.0f;
  }
  su_vel_initialized = false;
}

// ==============================
// 위치 → 속도 계산 (World)
//  - state->position.x/y/z [m]
//  - dt: main loop dt [s] (예: 1/1000)
// ==============================
void suVelFromPosUpdate(const state_t *state, float dt)
{
  float px = state->position.x;
  float py = state->position.y;
  float pz = state->position.z;

  if (!isfinite(px)) px = 0.0f;
  if (!isfinite(py)) py = 0.0f;
  if (!isfinite(pz)) pz = 0.0f;

  // 초기 구간 또는 dt 튀는 구간은 리셋
  if (!su_vel_initialized || dt <= 1e-5f || dt > 0.1f) {
    su_pos_prev[0] = px;
    su_pos_prev[1] = py;
    su_pos_prev[2] = pz;
    su_vel_lpf[0]  = 0.0f;
    su_vel_lpf[1]  = 0.0f;
    su_vel_lpf[2]  = 0.0f;
    su_vel_initialized = true;
    return;
  }

  // finite difference: v = Δp / dt
  float vx_raw = (px - su_pos_prev[0]) / dt;
  float vy_raw = (py - su_pos_prev[1]) / dt;
  float vz_raw = (pz - su_pos_prev[2]) / dt;

  su_pos_prev[0] = px;
  su_pos_prev[1] = py;
  su_pos_prev[2] = pz;

  // 1st-order LPF (cutoff ≈ 5 Hz)
  const float TWO_PI = 6.28318530718f;
  const float fc     = 2.0f;  // [Hz]
  float alpha_v = 1.0f - expf(-TWO_PI * fc * dt);

  if (alpha_v < 0.0f) alpha_v = 0.0f;
  if (alpha_v > 1.0f) alpha_v = 1.0f;

  su_vel_lpf[0] = lpf1(su_vel_lpf[0], vx_raw, alpha_v);
  su_vel_lpf[1] = lpf1(su_vel_lpf[1], vy_raw, alpha_v);
  su_vel_lpf[2] = lpf1(su_vel_lpf[2], vz_raw, alpha_v);
}

// ==============================
// World-frame velocity 읽기
// ==============================
void suVelFromPosGetWorld(float out_vW[3])
{
  if (!out_vW) return;

  out_vW[0] = su_vel_lpf[0];
  out_vW[1] = su_vel_lpf[1];
  out_vW[2] = su_vel_lpf[2];

  if (!isfinite(out_vW[0])) out_vW[0] = 0.0f;
  if (!isfinite(out_vW[1])) out_vW[1] = 0.0f;
  if (!isfinite(out_vW[2])) out_vW[2] = 0.0f;
}

// ==============================
// 로그 그룹 (ROS2에서 velocity만 따로 로깅용)
// ==============================
LOG_GROUP_START(suVelFromPos)

LOG_ADD(LOG_FLOAT, vx, &su_vel_lpf[0])   // [m/s]
LOG_ADD(LOG_FLOAT, vy, &su_vel_lpf[1])   // [m/s]
LOG_ADD(LOG_FLOAT, vz, &su_vel_lpf[2])   // [m/s]

LOG_GROUP_STOP(suVelFromPos)
