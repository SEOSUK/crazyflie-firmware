#include "platform_defaults.h"
#include "param.h"
#include "su_params.h"
#include "su_wrench_observer.h"

// ========= 전역 공유 파라미터 정의 (단일 소스) =========
// 플랫폼/모델
float su_mass            = CF_MASS;      // [kg] 원래는 0.0393


// Wrench observer / MOB 관련
float su_Kf              = 30.0f;        // [1/s] linear momentum observer gain
float su_Ktau            = 10.0f;        // [1/s] angular momentum observer gain
float su_Kp              = 10.9544511501f; // [1/s] translational momentum correction gain
float su_Kh              = 6.32f;        // [1/s] rotational momentum correction gain
float su_Keps            = 0.3f;         // [1/s] consistency residual 보정 이득
uint8_t su_zero_bias     = 0;            // MOB output bias capture trigger
float su_com_offset_x    = 0.0f;         // [m] body-frame CoM offset x
float su_com_offset_y    = 0.0f;         // [m] body-frame CoM offset y
float su_com_offset_z    = 0.0f;         // [m] body-frame CoM offset z
float su_r_offset_x      = 0.1f;         // [m] body-frame point-contact offset x
float su_r_offset_y      = 0.0f;         // [m] body-frame point-contact offset y
float su_r_offset_z      = 0.04f;        // [m] body-frame point-contact offset z
uint8_t su_consistency_mode = 2;         // 0=None, 1=Residual, 2=Both
uint8_t su_traj1_shape    = 1;           // 0=None, 1=Circle, 2=Square
float su_traj1_size_x     = 0.30f;       // [m]
float su_traj1_size_y     = 0.30f;       // [m]
float su_traj1_period_s   = 6.0f;        // [s]
uint8_t su_normal_estimation = 1;        // 0: fixed normal, 1: enable force-dominant normal estimator
float su_normal_beta      = 3.0f;        // [1/s] normal-axis memory decay
float su_normal_epsilon_g = 0.003f;      // [m^2/s^2] velocity projection regularization
float su_normal_epsilon_f = 0.01f;       // [N] minimum force evidence norm
float su_g_nf             = 1.0f;        // normal force tracking gain
float su_g_nv             = 2.0f;        // normal velocity damping gain
float su_nu_n_bar         = 0.08f;       // [m/s] symmetric saturation of normal velocity command
float su_epsilon_f_min    = 0.005f;      // [N] lower threshold where yaw-alignment smoothing starts
float su_epsilon_f_max    = 0.010f;      // [N] upper threshold where yaw-alignment smoothing saturates
float su_yaw_force_lpf_hz = 2.0f;        // [Hz] LPF cutoff for force-direction yaw alignment

static void suZeroBiasCallback(void)
{
  if (su_zero_bias == 0) {
    return;
  }

  suWrenchObserverRequestZeroBias();
  su_zero_bias = 0;
}

// ========= PARAM 등록 =========
// PARAM_GROUP_START(su_platform)
// // Platform / model parameters
// PARAM_ADD(PARAM_FLOAT, mass, &su_mass)
// PARAM_GROUP_STOP(su_platform)

// // Wrench/MOB 파라미터: 기존 su_wrench 그룹명 유지(로그/툴 호환성)
PARAM_GROUP_START(su_wrench)
PARAM_ADD(PARAM_FLOAT, mass,            &su_mass)
// 관측 이득
PARAM_ADD(PARAM_FLOAT, Keps,            &su_Keps)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, zeroBias, &su_zero_bias, suZeroBiasCallback)
PARAM_ADD(PARAM_FLOAT, comOffX,         &su_com_offset_x)
PARAM_ADD(PARAM_FLOAT, comOffY,         &su_com_offset_y)
PARAM_ADD(PARAM_FLOAT, comOffZ,         &su_com_offset_z)
PARAM_ADD(PARAM_FLOAT, rOffX,           &su_r_offset_x)
PARAM_ADD(PARAM_FLOAT, rOffY,           &su_r_offset_y)
PARAM_ADD(PARAM_FLOAT, rOffZ,           &su_r_offset_z)
PARAM_ADD(PARAM_UINT8, consistencyMode, &su_consistency_mode)
PARAM_GROUP_STOP(su_wrench)

PARAM_GROUP_START(su_position)
PARAM_ADD(PARAM_UINT8, traj1Shape,      &su_traj1_shape)
PARAM_ADD(PARAM_FLOAT, traj1SizeX,      &su_traj1_size_x)
PARAM_ADD(PARAM_FLOAT, traj1SizeY,      &su_traj1_size_y)
PARAM_ADD(PARAM_FLOAT, traj1Period,     &su_traj1_period_s)
PARAM_ADD(PARAM_UINT8, preloadEn,       &su_normal_estimation)
PARAM_ADD(PARAM_FLOAT, normBeta,        &su_normal_beta)
PARAM_ADD(PARAM_FLOAT, normEpsG,        &su_normal_epsilon_g)
PARAM_ADD(PARAM_FLOAT, normEpsF,        &su_normal_epsilon_f)
PARAM_ADD(PARAM_FLOAT, preloadGf,       &su_g_nf)
PARAM_ADD(PARAM_FLOAT, preloadGv,       &su_g_nv)
PARAM_ADD(PARAM_FLOAT, preloadNu,       &su_nu_n_bar)
PARAM_ADD(PARAM_FLOAT, epsilonFMin,     &su_epsilon_f_min)
PARAM_ADD(PARAM_FLOAT, epsilonFMax,     &su_epsilon_f_max)
PARAM_ADD(PARAM_FLOAT, yawForceLpfHz,   &su_yaw_force_lpf_hz)
PARAM_GROUP_STOP(su_position)
