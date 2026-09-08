// su_wrench_observer.c

#include "su_wrench_observer.h"
#include "su_params.h"

#include "platform_defaults.h"   // THRUST_MAX, THRUST2TORQUE, ARM_LENGTH
#include "physicalConstants.h"
#include "debug.h"
#include "log.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define SU_CONSISTENCY_NONE     0
#define SU_CONSISTENCY_RESIDUAL 1
#define SU_CONSISTENCY_BOTH     2
#define SU_OBSERVER_JXX         1.9e-5f
#define SU_OBSERVER_JYY         1.9e-5f
#define SU_OBSERVER_JZZ         3.0e-5f

static float su_motor_thrust_n[4];
static uint16_t su_motor_pwm_ratio[4];
static float su_body_force_n[3];
static float su_world_force_n[3];
static float su_body_torque_nm[3];
static float su_world_torque_nm[3];

static float su_state_vel_world[3];
static float su_vel_from_pos_world[3];
static float su_state_acc_world_mps2[3];
static float su_vel_used_world[3];
static float su_contact_point_vel_world[3];

static float su_gyro_body_rad_s[3];
static float su_r_offset_body_m[3];
static float su_r_offset_world_m[3];

static float su_rot_momentum_body[3];
static float su_rot_momentum_hat_body[3];
static float su_rot_momentum_err_body[3];
static float su_torque_l_hat_body[3];
static float su_torque_l_hat_world[3];
static float su_torque_l_hat_raw_world[3];
static float su_torque_bias_world[3];
static float su_torque_l_out_world[3];

static float su_lin_momentum_world[3];
static float su_lin_momentum_hat_world[3];
static float su_lin_momentum_err_world[3];
static float su_force_l_hat_raw_world[3];
static float su_force_bias_world[3];
static float su_force_l_hat_world[3];
static float su_force_l_hat_body[3];
static float su_lin_momentum_hat_none_world[3];
static float su_lin_momentum_err_none_world[3];
static float su_force_l_hat_none_world[3];
static float su_lin_momentum_hat_residual_world[3];
static float su_lin_momentum_err_residual_world[3];
static float su_force_l_hat_residual_world[3];

static float su_consistency_residual_world[3];
static float su_consistency_correction_world[3];

static uint32_t su_zero_bias_count = 0;
static bool su_zero_bias_pending = false;

static float clampFinite(float value)
{
  return isfinite(value) ? value : 0.0f;
}

static float sanitizeFinite(float value)
{
  return clampFinite(value);
}

static void vec3Set(float out[3], const float x, const float y, const float z)
{
  out[0] = x;
  out[1] = y;
  out[2] = z;
}

static void vec3Copy(float out[3], const float in[3])
{
  out[0] = in[0];
  out[1] = in[1];
  out[2] = in[2];
}

static void vec3Add(float out[3], const float a[3], const float b[3])
{
  out[0] = a[0] + b[0];
  out[1] = a[1] + b[1];
  out[2] = a[2] + b[2];
}

static void vec3Sub(float out[3], const float a[3], const float b[3])
{
  out[0] = a[0] - b[0];
  out[1] = a[1] - b[1];
  out[2] = a[2] - b[2];
}

static void vec3Scale(float out[3], const float in[3], const float scale)
{
  out[0] = in[0] * scale;
  out[1] = in[1] * scale;
  out[2] = in[2] * scale;
}

static void vec3ScaleAdd(float out[3], const float a[3], const float scale, const float b[3])
{
  out[0] = a[0] + scale * b[0];
  out[1] = a[1] + scale * b[1];
  out[2] = a[2] + scale * b[2];
}

static void vec3Cross(float out[3], const float a[3], const float b[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

static void quatToRotMat(const float qx, const float qy, const float qz, const float qw, float R[3][3])
{
  const float xx = qx * qx;
  const float yy = qy * qy;
  const float zz = qz * qz;
  const float xy = qx * qy;
  const float xz = qx * qz;
  const float yz = qy * qz;
  const float xw = qx * qw;
  const float yw = qy * qw;
  const float zw = qz * qw;

  R[0][0] = 1.0f - 2.0f * (yy + zz);
  R[0][1] = 2.0f * (xy - zw);
  R[0][2] = 2.0f * (xz + yw);

  R[1][0] = 2.0f * (xy + zw);
  R[1][1] = 1.0f - 2.0f * (xx + zz);
  R[1][2] = 2.0f * (yz - xw);

  R[2][0] = 2.0f * (xz - yw);
  R[2][1] = 2.0f * (yz + xw);
  R[2][2] = 1.0f - 2.0f * (xx + yy);
}

static void mat3MulVec(float out[3], const float R[3][3], const float v[3])
{
  out[0] = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
  out[1] = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
  out[2] = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
}

static void mat3TransposeMulVec(float out[3], const float R[3][3], const float v[3])
{
  out[0] = R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2];
  out[1] = R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2];
  out[2] = R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2];
}

static void sanitizeVec3(float vec[3])
{
  vec[0] = sanitizeFinite(vec[0]);
  vec[1] = sanitizeFinite(vec[1]);
  vec[2] = sanitizeFinite(vec[2]);
}

void suWrenchObserverRequestZeroBias(void)
{
  su_zero_bias_count++;
  su_zero_bias_pending = true;
  DEBUG_PRINT("SU Wrench observer zeroBias requested\n");
}

void suWrenchObserverInit(void)
{
  for (int i = 0; i < 4; ++i) {
    su_motor_thrust_n[i] = 0.0f;
    su_motor_pwm_ratio[i] = 0;
  }

  for (int i = 0; i < 3; ++i) {
    su_body_force_n[i] = 0.0f;
    su_world_force_n[i] = 0.0f;
    su_body_torque_nm[i] = 0.0f;
    su_world_torque_nm[i] = 0.0f;
    su_state_vel_world[i] = 0.0f;
    su_vel_from_pos_world[i] = 0.0f;
    su_state_acc_world_mps2[i] = 0.0f;
    su_vel_used_world[i] = 0.0f;
    su_contact_point_vel_world[i] = 0.0f;
    su_gyro_body_rad_s[i] = 0.0f;
    su_r_offset_body_m[i] = 0.0f;
    su_r_offset_world_m[i] = 0.0f;
    su_rot_momentum_body[i] = 0.0f;
    su_rot_momentum_hat_body[i] = 0.0f;
    su_rot_momentum_err_body[i] = 0.0f;
    su_torque_l_hat_body[i] = 0.0f;
    su_torque_l_hat_world[i] = 0.0f;
    su_torque_l_hat_raw_world[i] = 0.0f;
    su_torque_bias_world[i] = 0.0f;
    su_torque_l_out_world[i] = 0.0f;
    su_lin_momentum_world[i] = 0.0f;
    su_lin_momentum_hat_world[i] = 0.0f;
    su_lin_momentum_err_world[i] = 0.0f;
    su_force_l_hat_raw_world[i] = 0.0f;
    su_force_bias_world[i] = 0.0f;
    su_force_l_hat_world[i] = 0.0f;
    su_force_l_hat_body[i] = 0.0f;
    su_lin_momentum_hat_none_world[i] = 0.0f;
    su_lin_momentum_err_none_world[i] = 0.0f;
    su_force_l_hat_none_world[i] = 0.0f;
    su_lin_momentum_hat_residual_world[i] = 0.0f;
    su_lin_momentum_err_residual_world[i] = 0.0f;
    su_force_l_hat_residual_world[i] = 0.0f;
    su_consistency_residual_world[i] = 0.0f;
    su_consistency_correction_world[i] = 0.0f;
  }

  su_zero_bias_count = 0;
  su_zero_bias_pending = false;

  DEBUG_PRINT("SU Wrench observer initialized\n");
}

void suWrenchObserverUpdate(const state_t *state,
                            const motors_thrust_uncapped_t *motorThrustReq,
                            const motors_thrust_pwm_t *motorPwm,
                            const Axis3f *gyro_deg_s,
                            const float vel_from_pos_world[3],
                            float dt)
{
  if (!state || !motorThrustReq || !motorPwm) {
    return;
  }
  const float thrust_to_n = THRUST_MAX / (float)UINT16_MAX;
  const float gravity_world[3] = {0.0f, 0.0f, -su_mass * 9.81f};

  const float f1 = thrust_to_n * (float)motorThrustReq->motors.m1;
  const float f2 = thrust_to_n * (float)motorThrustReq->motors.m2;
  const float f3 = thrust_to_n * (float)motorThrustReq->motors.m3;
  const float f4 = thrust_to_n * (float)motorThrustReq->motors.m4;

  // NOTE: motorThrustReq is the battery-compensated uncapped request value,
  // matching the same raw PWM-scale quantity exposed as motor.m1req..m4req.

  su_motor_thrust_n[0] = sanitizeFinite(f1);
  su_motor_thrust_n[1] = sanitizeFinite(f2);
  su_motor_thrust_n[2] = sanitizeFinite(f3);
  su_motor_thrust_n[3] = sanitizeFinite(f4);

  su_motor_pwm_ratio[0] = motorPwm->motors.m1;
  su_motor_pwm_ratio[1] = motorPwm->motors.m2;
  su_motor_pwm_ratio[2] = motorPwm->motors.m3;
  su_motor_pwm_ratio[3] = motorPwm->motors.m4;

  const float Fz_body = su_motor_thrust_n[0] + su_motor_thrust_n[1] +
                        su_motor_thrust_n[2] + su_motor_thrust_n[3];

  su_body_force_n[0] = 0.0f;
  su_body_force_n[1] = 0.0f;
  su_body_force_n[2] = sanitizeFinite(Fz_body);

  const float arm = 0.707106781f * ARM_LENGTH;
  su_body_torque_nm[0] = sanitizeFinite(arm * ((su_motor_thrust_n[2] + su_motor_thrust_n[3]) -
                                                (su_motor_thrust_n[0] + su_motor_thrust_n[1])));
  su_body_torque_nm[1] = sanitizeFinite(arm * ((su_motor_thrust_n[1] + su_motor_thrust_n[2]) -
                                                (su_motor_thrust_n[0] + su_motor_thrust_n[3])));
  su_body_torque_nm[2] = sanitizeFinite(THRUST2TORQUE * (-su_motor_thrust_n[0] + su_motor_thrust_n[1] -
                                                         su_motor_thrust_n[2] + su_motor_thrust_n[3]));

  const float com_offset_body[3] = {su_com_offset_x, su_com_offset_y, su_com_offset_z};
  float tau_com_body[3];
  vec3Cross(tau_com_body, com_offset_body, su_body_force_n);
  vec3Add(su_body_torque_nm, su_body_torque_nm, tau_com_body);
  sanitizeVec3(su_body_torque_nm);

  float qx = sanitizeFinite(state->attitudeQuaternion.x);
  float qy = sanitizeFinite(state->attitudeQuaternion.y);
  float qz = sanitizeFinite(state->attitudeQuaternion.z);
  float qw = sanitizeFinite(state->attitudeQuaternion.w);

  const float q_norm = sqrtf(qx * qx + qy * qy + qz * qz + qw * qw);
  if (q_norm > 1e-6f) {
    qx /= q_norm;
    qy /= q_norm;
    qz /= q_norm;
    qw /= q_norm;
  } else {
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;
    qw = 1.0f;
  }

  float R[3][3];
  quatToRotMat(qx, qy, qz, qw, R);

  mat3MulVec(su_world_force_n, R, su_body_force_n);
  sanitizeVec3(su_world_force_n);
  mat3MulVec(su_world_torque_nm, R, su_body_torque_nm);
  sanitizeVec3(su_world_torque_nm);

  su_state_vel_world[0] = sanitizeFinite(state->velocity.x);
  su_state_vel_world[1] = sanitizeFinite(state->velocity.y);
  su_state_vel_world[2] = sanitizeFinite(state->velocity.z);

  if (vel_from_pos_world) {
    su_vel_from_pos_world[0] = sanitizeFinite(vel_from_pos_world[0]);
    su_vel_from_pos_world[1] = sanitizeFinite(vel_from_pos_world[1]);
    su_vel_from_pos_world[2] = sanitizeFinite(vel_from_pos_world[2]);
  } else {
    su_vel_from_pos_world[0] = 0.0f;
    su_vel_from_pos_world[1] = 0.0f;
    su_vel_from_pos_world[2] = 0.0f;
  }

  // Crazyflie firmware state.acc is stored in Gs in the world frame.
  // Per firmware convention, z is gravity-compensated.
  const float g = 9.81f;
  su_state_acc_world_mps2[0] = sanitizeFinite(state->acc.x * g);
  su_state_acc_world_mps2[1] = sanitizeFinite(state->acc.y * g);
  su_state_acc_world_mps2[2] = sanitizeFinite(state->acc.z * g);

  if (vel_from_pos_world) {
    vec3Copy(su_vel_used_world, su_vel_from_pos_world);
  } else {
    vec3Copy(su_vel_used_world, su_state_vel_world);
  }

  vec3Set(su_gyro_body_rad_s,
          gyro_deg_s ? sanitizeFinite(gyro_deg_s->x * (M_PI_F / 180.0f)) : 0.0f,
          gyro_deg_s ? sanitizeFinite(gyro_deg_s->y * (M_PI_F / 180.0f)) : 0.0f,
          gyro_deg_s ? sanitizeFinite(gyro_deg_s->z * (M_PI_F / 180.0f)) : 0.0f);

  vec3Set(su_r_offset_body_m, su_r_offset_x, su_r_offset_y, su_r_offset_z);
  mat3MulVec(su_r_offset_world_m, R, su_r_offset_body_m);
  sanitizeVec3(su_r_offset_world_m);

  float omega_cross_r_body[3];
  float contact_offset_vel_world[3];
  vec3Cross(omega_cross_r_body, su_gyro_body_rad_s, su_r_offset_body_m);
  mat3MulVec(contact_offset_vel_world, R, omega_cross_r_body);
  vec3Add(su_contact_point_vel_world, su_vel_used_world, contact_offset_vel_world);
  sanitizeVec3(su_contact_point_vel_world);

  su_rot_momentum_body[0] = sanitizeFinite(SU_OBSERVER_JXX * su_gyro_body_rad_s[0]);
  su_rot_momentum_body[1] = sanitizeFinite(SU_OBSERVER_JYY * su_gyro_body_rad_s[1]);
  su_rot_momentum_body[2] = sanitizeFinite(SU_OBSERVER_JZZ * su_gyro_body_rad_s[2]);

  su_lin_momentum_world[0] = sanitizeFinite(su_mass * su_vel_used_world[0]);
  su_lin_momentum_world[1] = sanitizeFinite(su_mass * su_vel_used_world[1]);
  su_lin_momentum_world[2] = sanitizeFinite(su_mass * su_vel_used_world[2]);

  vec3Sub(su_rot_momentum_err_body, su_rot_momentum_body, su_rot_momentum_hat_body);

  float omega_cross_hhat[3];
  float rot_momentum_hat_dot[3];
  vec3Cross(omega_cross_hhat, su_gyro_body_rad_s, su_rot_momentum_hat_body);
  for (int i = 0; i < 3; ++i) {
    rot_momentum_hat_dot[i] = -omega_cross_hhat[i] + su_body_torque_nm[i] + su_torque_l_hat_body[i] +
                              su_Kh * su_rot_momentum_err_body[i];
  }

  float torque_l_hat_dot_body[3];
  vec3Scale(torque_l_hat_dot_body, su_rot_momentum_err_body, su_Ktau);

  vec3ScaleAdd(su_rot_momentum_hat_body, su_rot_momentum_hat_body, dt, rot_momentum_hat_dot);
  vec3ScaleAdd(su_torque_l_hat_body, su_torque_l_hat_body, dt, torque_l_hat_dot_body);
  sanitizeVec3(su_rot_momentum_hat_body);
  sanitizeVec3(su_torque_l_hat_body);

  mat3MulVec(su_torque_l_hat_world, R, su_torque_l_hat_body);
  sanitizeVec3(su_torque_l_hat_world);
  vec3Copy(su_torque_l_hat_raw_world, su_torque_l_hat_world);

  vec3Sub(su_lin_momentum_err_none_world, su_lin_momentum_world, su_lin_momentum_hat_none_world);

  float lin_momentum_hat_none_dot[3];
  for (int i = 0; i < 3; ++i) {
    lin_momentum_hat_none_dot[i] = gravity_world[i] + su_world_force_n[i] + su_force_l_hat_none_world[i] +
                                   su_Kp * su_lin_momentum_err_none_world[i];
  }

  vec3ScaleAdd(su_lin_momentum_hat_none_world, su_lin_momentum_hat_none_world, dt, lin_momentum_hat_none_dot);
  sanitizeVec3(su_lin_momentum_hat_none_world);

  float force_l_hat_none_dot_world[3];
  vec3Scale(force_l_hat_none_dot_world, su_lin_momentum_err_none_world, su_Kf);
  vec3ScaleAdd(su_force_l_hat_none_world, su_force_l_hat_none_world, dt, force_l_hat_none_dot_world);
  sanitizeVec3(su_force_l_hat_none_world);

  vec3Sub(su_lin_momentum_err_residual_world, su_lin_momentum_world, su_lin_momentum_hat_residual_world);

  float lin_momentum_hat_residual_dot[3];
  for (int i = 0; i < 3; ++i) {
    lin_momentum_hat_residual_dot[i] = gravity_world[i] + su_world_force_n[i] + su_force_l_hat_residual_world[i] +
                                       su_Kp * su_lin_momentum_err_residual_world[i];
  }

  vec3ScaleAdd(su_lin_momentum_hat_residual_world, su_lin_momentum_hat_residual_world, dt, lin_momentum_hat_residual_dot);
  sanitizeVec3(su_lin_momentum_hat_residual_world);

  float r_cross_fhat_world[3];
  vec3Cross(r_cross_fhat_world, su_r_offset_world_m, su_force_l_hat_residual_world);
  vec3Sub(su_consistency_residual_world, su_torque_l_hat_world, r_cross_fhat_world);

  vec3Cross(su_consistency_correction_world, su_consistency_residual_world, su_r_offset_world_m);
  vec3Scale(su_consistency_correction_world, su_consistency_correction_world, su_Keps);

  float force_l_hat_residual_dot_world[3];
  vec3Scale(force_l_hat_residual_dot_world, su_lin_momentum_err_residual_world, su_Kf);
  vec3Add(force_l_hat_residual_dot_world, force_l_hat_residual_dot_world, su_consistency_correction_world);
  vec3ScaleAdd(su_force_l_hat_residual_world, su_force_l_hat_residual_world, dt, force_l_hat_residual_dot_world);
  sanitizeVec3(su_force_l_hat_residual_world);

  switch (su_consistency_mode) {
    case SU_CONSISTENCY_RESIDUAL:
      vec3Copy(su_lin_momentum_hat_world, su_lin_momentum_hat_residual_world);
      vec3Copy(su_lin_momentum_err_world, su_lin_momentum_err_residual_world);
      vec3Copy(su_force_l_hat_raw_world, su_force_l_hat_residual_world);
      break;
    case SU_CONSISTENCY_BOTH:
      vec3Copy(su_lin_momentum_hat_world, su_lin_momentum_hat_residual_world);
      vec3Copy(su_lin_momentum_err_world, su_lin_momentum_err_residual_world);
      vec3Copy(su_force_l_hat_raw_world, su_force_l_hat_residual_world);
      break;
    case SU_CONSISTENCY_NONE:
    default:
      vec3Copy(su_lin_momentum_hat_world, su_lin_momentum_hat_none_world);
      vec3Copy(su_lin_momentum_err_world, su_lin_momentum_err_none_world);
      vec3Copy(su_force_l_hat_raw_world, su_force_l_hat_none_world);
      break;
  }

  if (su_zero_bias_pending) {
    vec3Copy(su_force_bias_world, su_force_l_hat_raw_world);
    vec3Copy(su_torque_bias_world, su_torque_l_hat_raw_world);
    su_zero_bias_pending = false;
  }

  vec3Sub(su_force_l_hat_world, su_force_l_hat_raw_world, su_force_bias_world);
  vec3Sub(su_torque_l_out_world, su_torque_l_hat_raw_world, su_torque_bias_world);
  su_force_l_hat_world[2] *= 0.1f;
  sanitizeVec3(su_force_l_hat_world);
  sanitizeVec3(su_torque_l_out_world);

  mat3TransposeMulVec(su_force_l_hat_body, R, su_force_l_hat_world);
  sanitizeVec3(su_force_l_hat_body);
}

void suWrenchObserverGetWorldForce(float outF[3])
{
  if (!outF) {
    return;
  }

  outF[0] = su_force_l_hat_world[0];
  outF[1] = su_force_l_hat_world[1];
  outF[2] = su_force_l_hat_world[2];
}

void suWrenchObserverGetWorldTorque(float outTau[3])
{
  if (!outTau) {
    return;
  }

  outTau[0] = su_torque_l_out_world[0];
  outTau[1] = su_torque_l_out_world[1];
  outTau[2] = su_torque_l_out_world[2];
}

void suWrenchObserverGetWorldInputForce(float outF[3])
{
  if (!outF) {
    return;
  }

  outF[0] = su_world_force_n[0];
  outF[1] = su_world_force_n[1];
  outF[2] = su_world_force_n[2];
}

void suWrenchObserverGetWorldInputTorque(float outTau[3])
{
  if (!outTau) {
    return;
  }

  outTau[0] = su_world_torque_nm[0];
  outTau[1] = su_world_torque_nm[1];
  outTau[2] = su_world_torque_nm[2];
}

void suWrenchObserverGetContactOffsetWorld(float outR[3])
{
  if (!outR) {
    return;
  }

  outR[0] = su_r_offset_world_m[0];
  outR[1] = su_r_offset_world_m[1];
  outR[2] = su_r_offset_world_m[2];
}

void suWrenchObserverGetStateVelocityWorld(float outV[3])
{
  if (!outV) {
    return;
  }

  outV[0] = su_state_vel_world[0];
  outV[1] = su_state_vel_world[1];
  outV[2] = su_state_vel_world[2];
}

void suWrenchObserverGetContactPointVelocityWorld(float outV[3])
{
  if (!outV) {
    return;
  }

  outV[0] = su_contact_point_vel_world[0];
  outV[1] = su_contact_point_vel_world[1];
  outV[2] = su_contact_point_vel_world[2];
}

LOG_GROUP_START(suWrenchObs)
// Fill momentum observer values //
// LOG_ADD(LOG_FLOAT, omgX, &su_gyro_body_rad_s[0])       // rad/s, body angular velocity
// LOG_ADD(LOG_FLOAT, omgY, &su_gyro_body_rad_s[1])       // rad/s, body angular velocity
// LOG_ADD(LOG_FLOAT, omgZ, &su_gyro_body_rad_s[2])       // rad/s, body angular velocity

// LOG_ADD(LOG_FLOAT, rOffBx, &su_r_offset_body_m[0])     // m, body-frame point-contact offset
// LOG_ADD(LOG_FLOAT, rOffBy, &su_r_offset_body_m[1])     // m, body-frame point-contact offset
// LOG_ADD(LOG_FLOAT, rOffBz, &su_r_offset_body_m[2])     // m, body-frame point-contact offset
// LOG_ADD(LOG_FLOAT, rOffWx, &su_r_offset_world_m[0])    // m, world-frame point-contact offset
// LOG_ADD(LOG_FLOAT, rOffWy, &su_r_offset_world_m[1])    // m, world-frame point-contact offset
// LOG_ADD(LOG_FLOAT, rOffWz, &su_r_offset_world_m[2])    // m, world-frame point-contact offset

// LOG_ADD(LOG_FLOAT, hmBx, &su_rot_momentum_body[0])     // N*m*s, measured body angular momentum
// LOG_ADD(LOG_FLOAT, hmBy, &su_rot_momentum_body[1])     // N*m*s, measured body angular momentum
// LOG_ADD(LOG_FLOAT, hmBz, &su_rot_momentum_body[2])     // N*m*s, measured body angular momentum
// LOG_ADD(LOG_FLOAT, hmHatX, &su_rot_momentum_hat_body[0]) // N*m*s, estimated body angular momentum
// LOG_ADD(LOG_FLOAT, hmHatY, &su_rot_momentum_hat_body[1]) // N*m*s, estimated body angular momentum
// LOG_ADD(LOG_FLOAT, hmHatZ, &su_rot_momentum_hat_body[2]) // N*m*s, estimated body angular momentum
// LOG_ADD(LOG_FLOAT, hmErrX, &su_rot_momentum_err_body[0]) // N*m*s, body angular momentum residual
// LOG_ADD(LOG_FLOAT, hmErrY, &su_rot_momentum_err_body[1]) // N*m*s, body angular momentum residual
// LOG_ADD(LOG_FLOAT, hmErrZ, &su_rot_momentum_err_body[2]) // N*m*s, body angular momentum residual

// LOG_ADD(LOG_FLOAT, tauLBx, &su_torque_l_hat_body[0])   // N*m, estimated lumped torque in body frame
// LOG_ADD(LOG_FLOAT, tauLBy, &su_torque_l_hat_body[1])   // N*m, estimated lumped torque in body frame
// LOG_ADD(LOG_FLOAT, tauLBz, &su_torque_l_hat_body[2])   // N*m, estimated lumped torque in body frame
LOG_ADD(LOG_FLOAT, tauLWx, &su_torque_l_out_world[0])  // N*m, bias-compensated lumped torque in world frame
LOG_ADD(LOG_FLOAT, tauLWy, &su_torque_l_out_world[1])  // N*m, bias-compensated lumped torque in world frame
LOG_ADD(LOG_FLOAT, tauLWz, &su_torque_l_out_world[2])  // N*m, bias-compensated lumped torque in world frame
// LOG_ADD(LOG_FLOAT, tauBiasWx, &su_torque_bias_world[0])  // N*m, captured torque bias in world frame
// LOG_ADD(LOG_FLOAT, tauBiasWy, &su_torque_bias_world[1])  // N*m, captured torque bias in world frame
// LOG_ADD(LOG_FLOAT, tauBiasWz, &su_torque_bias_world[2])  // N*m, captured torque bias in world frame

// LOG_ADD(LOG_FLOAT, pWx, &su_lin_momentum_world[0])     // N*s, measured world linear momentum
// LOG_ADD(LOG_FLOAT, pWy, &su_lin_momentum_world[1])     // N*s, measured world linear momentum
// LOG_ADD(LOG_FLOAT, pWz, &su_lin_momentum_world[2])     // N*s, measured world linear momentum
// LOG_ADD(LOG_FLOAT, pHatX, &su_lin_momentum_hat_world[0]) // N*s, estimated world linear momentum
// LOG_ADD(LOG_FLOAT, pHatY, &su_lin_momentum_hat_world[1]) // N*s, estimated world linear momentum
// LOG_ADD(LOG_FLOAT, pHatZ, &su_lin_momentum_hat_world[2]) // N*s, estimated world linear momentum
// LOG_ADD(LOG_FLOAT, pErrX, &su_lin_momentum_err_world[0]) // N*s, world linear momentum residual
// LOG_ADD(LOG_FLOAT, pErrY, &su_lin_momentum_err_world[1]) // N*s, world linear momentum residual
// LOG_ADD(LOG_FLOAT, pErrZ, &su_lin_momentum_err_world[2]) // N*s, world linear momentum residual

LOG_ADD(LOG_FLOAT, fHatNWx, &su_force_l_hat_none_world[0])   // N, momentum-only lumped force estimate in world frame
LOG_ADD(LOG_FLOAT, fHatNWy, &su_force_l_hat_none_world[1])   // N, momentum-only lumped force estimate in world frame
LOG_ADD(LOG_FLOAT, fHatNWz, &su_force_l_hat_none_world[2])   // N, momentum-only lumped force estimate in world frame
LOG_ADD(LOG_FLOAT, fHatRWx, &su_force_l_hat_residual_world[0]) // N, momentum+consistency lumped force estimate in world frame
LOG_ADD(LOG_FLOAT, fHatRWy, &su_force_l_hat_residual_world[1]) // N, momentum+consistency lumped force estimate in world frame
LOG_ADD(LOG_FLOAT, fHatRWz, &su_force_l_hat_residual_world[2]) // N, momentum+consistency lumped force estimate in world frame
LOG_ADD(LOG_FLOAT, fHatWx, &su_force_l_hat_world[0])   // N, bias-compensated lumped force in world frame
LOG_ADD(LOG_FLOAT, fHatWy, &su_force_l_hat_world[1])   // N, bias-compensated lumped force in world frame
LOG_ADD(LOG_FLOAT, fHatWz, &su_force_l_hat_world[2])   // N, bias-compensated lumped force in world frame
// LOG_ADD(LOG_FLOAT, fBiasWx, &su_force_bias_world[0])   // N, captured force bias in world frame
// LOG_ADD(LOG_FLOAT, fBiasWy, &su_force_bias_world[1])   // N, captured force bias in world frame
// LOG_ADD(LOG_FLOAT, fBiasWz, &su_force_bias_world[2])   // N, captured force bias in world frame
// LOG_ADD(LOG_FLOAT, fHatBx, &su_force_l_hat_body[0])    // N, estimated lumped force in body frame
// LOG_ADD(LOG_FLOAT, fHatBy, &su_force_l_hat_body[1])    // N, estimated lumped force in body frame
// LOG_ADD(LOG_FLOAT, fHatBz, &su_force_l_hat_body[2])    // N, estimated lumped force in body frame

// LOG_ADD(LOG_FLOAT, corrFx, &su_consistency_correction_world[0]) // N/s, consistency correction term
// LOG_ADD(LOG_FLOAT, corrFy, &su_consistency_correction_world[1]) // N/s, consistency correction term
// LOG_ADD(LOG_FLOAT, corrFz, &su_consistency_correction_world[2]) // N/s, consistency correction term
// LOG_ADD(LOG_FLOAT, usedVx, &su_vel_used_world[0])      // m/s, velocity used for momentum observer
// LOG_ADD(LOG_FLOAT, usedVy, &su_vel_used_world[1])      // m/s, velocity used for momentum observer
// LOG_ADD(LOG_FLOAT, usedVz, &su_vel_used_world[2])      // m/s, velocity used for momentum observer

// for unit check. keep this. //
// 각 모터 추력. saturation 0.2N. 꽤 잘 fitting 됨
LOG_ADD(LOG_FLOAT, f1, &su_motor_thrust_n[0])           // N, motor 1 thrust command (pre-battery-comp, pre-cap)
LOG_ADD(LOG_FLOAT, f2, &su_motor_thrust_n[1])           // N, motor 2 thrust command (pre-battery-comp, pre-cap)
LOG_ADD(LOG_FLOAT, f3, &su_motor_thrust_n[2])           // N, motor 3 thrust command (pre-battery-comp, pre-cap)
LOG_ADD(LOG_FLOAT, f4, &su_motor_thrust_n[3])           // N, motor 4 thrust command (pre-battery-comp, pre-cap)
LOG_ADD(LOG_FLOAT, tauInBx, &su_body_torque_nm[0])      // N*m, body-frame input torque including CoM-offset contribution
LOG_ADD(LOG_FLOAT, tauInBy, &su_body_torque_nm[1])      // N*m, body-frame input torque including CoM-offset contribution
LOG_ADD(LOG_FLOAT, tauInBz, &su_body_torque_nm[2])      // N*m, body-frame input torque including CoM-offset contribution

LOG_ADD(LOG_FLOAT, stateVx, &su_state_vel_world[0])     // m/s, world frame state.velocity
LOG_ADD(LOG_FLOAT, stateVy, &su_state_vel_world[1])     // m/s, world frame state.velocity
LOG_ADD(LOG_FLOAT, stateVz, &su_state_vel_world[2])     // m/s, world frame state.velocity

LOG_ADD(LOG_FLOAT, posVx, &su_vel_from_pos_world[0])    // m/s, world frame velocity from su_vel_from_pos
LOG_ADD(LOG_FLOAT, posVy, &su_vel_from_pos_world[1])    // m/s, world frame velocity from su_vel_from_pos
LOG_ADD(LOG_FLOAT, posVz, &su_vel_from_pos_world[2])    // m/s, world frame velocity from su_vel_from_pos

LOG_ADD(LOG_FLOAT, accWx, &su_state_acc_world_mps2[0])  // m/s^2, world frame state.acc
LOG_ADD(LOG_FLOAT, accWy, &su_state_acc_world_mps2[1])  // m/s^2, world frame state.acc
LOG_ADD(LOG_FLOAT, accWz, &su_state_acc_world_mps2[2])  // m/s^2, world frame state.acc (gravity removed by firmware convention)
LOG_GROUP_STOP(suWrenchObs)
