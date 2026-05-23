#include "platform_defaults.h"
#include "param.h"
#include "su_params.h"
#include "su_wrench_observer.h"

float su_cmd_fx          = 0.0f;       // 커맨드 Force [N]
// ========= 전역 공유 파라미터 정의 (단일 소스) =========
// 플랫폼/모델
float su_mass            = CF_MASS;      // [kg] 원래는 0.0393
float Jxx                = 1.9e-5f;      // [kg·m^2]
float Jyy                = 1.9e-5f;      // [kg·m^2]
float Jzz                = 3.0e-5f;      // [kg·m^2]


// Wrench observer / MOB 관련
float su_Kf              = 0.3f;         // [1/s] 선운동량 관측 이득
float su_Ktau            = 0.3f;         // [1/s] 각운동량 관측 이득
float su_Kp              = 0.3f;         // [1/s] 선운동량 상태 보정 이득
float su_Kh              = 0.3f;         // [1/s] 각운동량 상태 보정 이득
float su_Keps            = 0.3f;         // [1/s] consistency residual 보정 이득
float su_deadzone_F      = 0.000f;       // [N]   힘 deadzone
float su_deadzone_T      = 0.0000f;      // [N·m] 토크 deadzone
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
uint8_t su_traj2_shape    = 2;           // 0=None, 1=Circle, 2=Square
float su_traj2_size_x     = 0.40f;       // [m]
float su_traj2_size_y     = 0.40f;       // [m]
float su_traj2_period_s   = 8.0f;        // [s]

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
// PARAM_ADD(PARAM_FLOAT, Jxx,  &Jxx)
// PARAM_ADD(PARAM_FLOAT, Jyy,  &Jyy)
// PARAM_ADD(PARAM_FLOAT, Jzz,  &Jzz)
// PARAM_GROUP_STOP(su_platform)

// // Wrench/MOB 파라미터: 기존 su_wrench 그룹명 유지(로그/툴 호환성)
PARAM_GROUP_START(su_wrench)
PARAM_ADD(PARAM_FLOAT, mass,            &su_mass)
PARAM_ADD(PARAM_FLOAT, Jxx,             &Jxx)
PARAM_ADD(PARAM_FLOAT, Jyy,             &Jyy)
PARAM_ADD(PARAM_FLOAT, Jzz,             &Jzz)
// 관측 이득
PARAM_ADD(PARAM_FLOAT, Kf,              &su_Kf)
PARAM_ADD(PARAM_FLOAT, Ktau,            &su_Ktau)
PARAM_ADD(PARAM_FLOAT, Kp,              &su_Kp)
PARAM_ADD(PARAM_FLOAT, Kh,              &su_Kh)
PARAM_ADD(PARAM_FLOAT, Keps,            &su_Keps)
// Deadzone
PARAM_ADD(PARAM_FLOAT, deadzone_F,      &su_deadzone_F)
PARAM_ADD(PARAM_FLOAT, deadzone_T,      &su_deadzone_T)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, zeroBias, &su_zero_bias, suZeroBiasCallback)
PARAM_ADD(PARAM_FLOAT, cmd_fx,        &su_cmd_fx)
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
PARAM_ADD(PARAM_UINT8, traj2Shape,      &su_traj2_shape)
PARAM_ADD(PARAM_FLOAT, traj2SizeX,      &su_traj2_size_x)
PARAM_ADD(PARAM_FLOAT, traj2SizeY,      &su_traj2_size_y)
PARAM_ADD(PARAM_FLOAT, traj2Period,     &su_traj2_period_s)
PARAM_GROUP_STOP(su_position)
