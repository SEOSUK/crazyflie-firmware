// su_wrench_observer.c

#include "su_wrench_observer.h"

#include "platform_defaults.h"   // THRUST_MAX, THRUST2TORQUE, ARM_LENGTH
#include "debug.h"
#include "log.h"

#include <math.h>
#include <stdint.h>

static float su_motor_thrust_n[4];
static uint16_t su_motor_pwm_ratio[4];
static float su_body_force_n[3];
static float su_world_force_n[3];
static float su_body_torque_nm[3];

static float su_state_vel_world[3];
static float su_vel_from_pos_world[3];
static float su_state_acc_world_mps2[3];

static uint32_t su_zero_bias_count = 0;

static float sanitizeFinite(float value)
{
  return isfinite(value) ? value : 0.0f;
}

void suWrenchObserverRequestZeroBias(void)
{
  su_zero_bias_count++;
  DEBUG_PRINT("SU Wrench observer zeroBias requested (logging-only mode)\n");
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
    su_state_vel_world[i] = 0.0f;
    su_vel_from_pos_world[i] = 0.0f;
    su_state_acc_world_mps2[i] = 0.0f;
  }

  su_zero_bias_count = 0;

  DEBUG_PRINT("SU Wrench observer initialized (logging-only SI mode)\n");
}

void suWrenchObserverUpdate(const state_t *state,
                            const motors_thrust_uncapped_t *motorThrustUncapped,
                            const motors_thrust_pwm_t *motorPwm,
                            const Axis3f *gyro_deg_s,
                            const float vel_from_pos_world[3])
{
  (void)gyro_deg_s;

  if (!state || !motorThrustUncapped || !motorPwm) {
    return;
  }

  const float thrust_to_n = THRUST_MAX / (float)UINT16_MAX;

  const float f1 = thrust_to_n * (float)motorThrustUncapped->motors.m1;
  const float f2 = thrust_to_n * (float)motorThrustUncapped->motors.m2;
  const float f3 = thrust_to_n * (float)motorThrustUncapped->motors.m3;
  const float f4 = thrust_to_n * (float)motorThrustUncapped->motors.m4;

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

  const float xx = qx * qx;
  const float yy = qy * qy;
  const float xz = qx * qz;
  const float yz = qy * qz;
  const float xw = qx * qw;
  const float yw = qy * qw;

  const float R13 = 2.0f * (xz + yw);
  const float R23 = 2.0f * (yz - xw);
  const float R33 = 1.0f - 2.0f * (xx + yy);

  su_world_force_n[0] = sanitizeFinite(R13 * Fz_body);
  su_world_force_n[1] = sanitizeFinite(R23 * Fz_body);
  su_world_force_n[2] = sanitizeFinite(R33 * Fz_body);

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
}

void suWrenchObserverGetWorldForce(float outF[3])
{
  if (!outF) {
    return;
  }

  outF[0] = su_world_force_n[0];
  outF[1] = su_world_force_n[1];
  outF[2] = su_world_force_n[2];
}

LOG_GROUP_START(suWrenchObs)
LOG_ADD(LOG_FLOAT, f1, &su_motor_thrust_n[0])           // N, motor 1 thrust command (pre-battery-comp, pre-cap)
LOG_ADD(LOG_FLOAT, f2, &su_motor_thrust_n[1])           // N, motor 2 thrust command (pre-battery-comp, pre-cap)
LOG_ADD(LOG_FLOAT, f3, &su_motor_thrust_n[2])           // N, motor 3 thrust command (pre-battery-comp, pre-cap)
LOG_ADD(LOG_FLOAT, f4, &su_motor_thrust_n[3])           // N, motor 4 thrust command (pre-battery-comp, pre-cap)

LOG_ADD(LOG_UINT16, pwm1, &su_motor_pwm_ratio[0])       // ratio, final actuator command after battery compensation and cap
LOG_ADD(LOG_UINT16, pwm2, &su_motor_pwm_ratio[1])       // ratio, final actuator command after battery compensation and cap
LOG_ADD(LOG_UINT16, pwm3, &su_motor_pwm_ratio[2])       // ratio, final actuator command after battery compensation and cap
LOG_ADD(LOG_UINT16, pwm4, &su_motor_pwm_ratio[3])       // ratio, final actuator command after battery compensation and cap

LOG_ADD(LOG_FLOAT, bodyFx, &su_body_force_n[0])         // N, body frame input force
LOG_ADD(LOG_FLOAT, bodyFy, &su_body_force_n[1])         // N, body frame input force
LOG_ADD(LOG_FLOAT, bodyFz, &su_body_force_n[2])         // N, body frame input force

LOG_ADD(LOG_FLOAT, worldFx, &su_world_force_n[0])       // N, world frame input force
LOG_ADD(LOG_FLOAT, worldFy, &su_world_force_n[1])       // N, world frame input force
LOG_ADD(LOG_FLOAT, worldFz, &su_world_force_n[2])       // N, world frame input force

LOG_ADD(LOG_FLOAT, bodyTx, &su_body_torque_nm[0])       // N*m, body frame input torque
LOG_ADD(LOG_FLOAT, bodyTy, &su_body_torque_nm[1])       // N*m, body frame input torque
LOG_ADD(LOG_FLOAT, bodyTz, &su_body_torque_nm[2])       // N*m, body frame input torque

LOG_ADD(LOG_FLOAT, stateVx, &su_state_vel_world[0])     // m/s, world frame state.velocity
LOG_ADD(LOG_FLOAT, stateVy, &su_state_vel_world[1])     // m/s, world frame state.velocity
LOG_ADD(LOG_FLOAT, stateVz, &su_state_vel_world[2])     // m/s, world frame state.velocity

LOG_ADD(LOG_FLOAT, posVx, &su_vel_from_pos_world[0])    // m/s, world frame velocity from su_vel_from_pos
LOG_ADD(LOG_FLOAT, posVy, &su_vel_from_pos_world[1])    // m/s, world frame velocity from su_vel_from_pos
LOG_ADD(LOG_FLOAT, posVz, &su_vel_from_pos_world[2])    // m/s, world frame velocity from su_vel_from_pos

LOG_ADD(LOG_FLOAT, accWx, &su_state_acc_world_mps2[0])  // m/s^2, world frame state.acc
LOG_ADD(LOG_FLOAT, accWy, &su_state_acc_world_mps2[1])  // m/s^2, world frame state.acc
LOG_ADD(LOG_FLOAT, accWz, &su_state_acc_world_mps2[2])  // m/s^2, world frame state.acc (gravity removed by firmware convention)

LOG_ADD(LOG_UINT32, zeroCnt, &su_zero_bias_count)       // count, zeroBias requests in logging-only mode
LOG_GROUP_STOP(suWrenchObs)
