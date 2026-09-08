#include "su_thrust_effectiveness.h"

#include "su_wrench_observer.h"

#include "debug.h"
#include "log.h"

#include <math.h>
#include <stdbool.h>

#define SU_THRUST_EFFECTIVENESS_GAMMA 20.0f
#define SU_THRUST_EFFECTIVENESS_RHO   0.1f
#define SU_THRUST_EFFECTIVENESS_INIT  1.0f
#define SU_THRUST_EFFECTIVENESS_EPS   1.0e-6f
#define SU_THRUST_EFFECTIVENESS_KF    5.0f
#define SU_THRUST_EFFECTIVENESS_KP    2.9544511501f
#define SU_THRUST_EFFECTIVENESS_KTAU  30.0f
#define SU_THRUST_EFFECTIVENESS_KPTAU 10.9544511501f
#define SU_THRUST_EFFECTIVENESS_ALPHA 1.0f

static float su_eta_hat = SU_THRUST_EFFECTIVENESS_INIT;
static float su_nominal_force_world[3] = {0.0f, 0.0f, 0.0f};
static float su_point_contact_residual_world[3] = {0.0f, 0.0f, 0.0f};
static float su_corrected_force_world[3] = {0.0f, 0.0f, 0.0f};

static float su_matched_force_signal_world[3] = {0.0f, 0.0f, 0.0f};
static float su_matched_force_dot_world[3] = {0.0f, 0.0f, 0.0f};
static float su_matched_force_output_world[3] = {0.0f, 0.0f, 0.0f};
static float su_matched_torque_signal_world[3] = {0.0f, 0.0f, 0.0f};
static float su_matched_torque_dot_world[3] = {0.0f, 0.0f, 0.0f};
static float su_matched_torque_output_world[3] = {0.0f, 0.0f, 0.0f};

static void vec3Copy(float out[3], const float in[3])
{
  out[0] = in[0];
  out[1] = in[1];
  out[2] = in[2];
}

static void vec3Scale(float out[3], const float in[3], const float scale)
{
  out[0] = in[0] * scale;
  out[1] = in[1] * scale;
  out[2] = in[2] * scale;
}

static void vec3Cross(float out[3], const float a[3], const float b[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

static void vec3Sub(float out[3], const float a[3], const float b[3])
{
  out[0] = a[0] - b[0];
  out[1] = a[1] - b[1];
  out[2] = a[2] - b[2];
}

static float vec3Dot(const float a[3], const float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static float sanitizePositive(float value, float fallback)
{
  if (!isfinite(value) || value <= 1.0e-9f) {
    return fallback;
  }
  return value;
}

static float clampFinite(float value)
{
  if (!isfinite(value)) {
    return 0.0f;
  }
  return value;
}

static void sanitizeVec3(float vec[3])
{
  vec[0] = clampFinite(vec[0]);
  vec[1] = clampFinite(vec[1]);
  vec[2] = clampFinite(vec[2]);
}

static void lpf1Vec3(float out[3], const float input[3], float alpha)
{
  for (int i = 0; i < 3; ++i) {
    out[i] = out[i] + alpha * (input[i] - out[i]);
  }
  sanitizeVec3(out);
}

static void updateMatchedSignal(float signal[3], float signalDot[3], float output[3],
                                const float input[3], const float dt,
                                const float stiffnessGain, const float dampingGain)
{
  for (int i = 0; i < 3; ++i) {
    const float signalDdot = fmaxf(0.0f, stiffnessGain) * (input[i] - signal[i]) -
                             fmaxf(0.0f, dampingGain) * signalDot[i];
    signalDot[i] = clampFinite(signalDot[i] + dt * signalDdot);
    signal[i] = clampFinite(signal[i] + dt * signalDot[i]);
  }
  lpf1Vec3(output, signal, SU_THRUST_EFFECTIVENESS_ALPHA);
}

void suThrustEffectivenessInit(void)
{
  su_eta_hat = SU_THRUST_EFFECTIVENESS_INIT;
  vec3Copy(su_nominal_force_world, (const float[3]){0.0f, 0.0f, 0.0f});
  vec3Copy(su_point_contact_residual_world, (const float[3]){0.0f, 0.0f, 0.0f});
  vec3Copy(su_corrected_force_world, (const float[3]){0.0f, 0.0f, 0.0f});
  vec3Copy(su_matched_force_signal_world, (const float[3]){0.0f, 0.0f, 0.0f});
  vec3Copy(su_matched_force_dot_world, (const float[3]){0.0f, 0.0f, 0.0f});
  vec3Copy(su_matched_force_output_world, (const float[3]){0.0f, 0.0f, 0.0f});
  vec3Copy(su_matched_torque_signal_world, (const float[3]){0.0f, 0.0f, 0.0f});
  vec3Copy(su_matched_torque_dot_world, (const float[3]){0.0f, 0.0f, 0.0f});
  vec3Copy(su_matched_torque_output_world, (const float[3]){0.0f, 0.0f, 0.0f});
  DEBUG_PRINT("SU thrust effectiveness estimator initialized\n");
}

void suThrustEffectivenessUpdate(const state_t *state,
                                const motors_thrust_uncapped_t *motorThrustReq,
                                const Axis3f *gyro_deg_s,
                                const float vel_from_pos_world[3],
                                float dt)
{
  (void)state;
  (void)motorThrustReq;
  (void)gyro_deg_s;
  (void)vel_from_pos_world;

  if (!isfinite(dt) || dt <= 0.0f) {
    return;
  }

  float contactOffsetWorld[3];
  float nominalForceWorld[3];
  float nominalTorqueWorld[3];
  float lumpedForceWorld[3];
  float lumpedTorqueWorld[3];

  suWrenchObserverGetContactOffsetWorld(contactOffsetWorld);
  suWrenchObserverGetWorldInputForce(nominalForceWorld);
  suWrenchObserverGetWorldInputTorque(nominalTorqueWorld);
  suWrenchObserverGetWorldForce(lumpedForceWorld);
  suWrenchObserverGetWorldTorque(lumpedTorqueWorld);
  sanitizeVec3(contactOffsetWorld);
  sanitizeVec3(nominalForceWorld);
  sanitizeVec3(nominalTorqueWorld);
  sanitizeVec3(lumpedForceWorld);
  sanitizeVec3(lumpedTorqueWorld);

  updateMatchedSignal(su_matched_force_signal_world, su_matched_force_dot_world,
                      su_matched_force_output_world, nominalForceWorld, dt,
                      SU_THRUST_EFFECTIVENESS_KF, SU_THRUST_EFFECTIVENESS_KP);
  updateMatchedSignal(su_matched_torque_signal_world, su_matched_torque_dot_world,
                      su_matched_torque_output_world, nominalTorqueWorld, dt,
                      SU_THRUST_EFFECTIVENESS_KTAU, SU_THRUST_EFFECTIVENESS_KPTAU);
  vec3Copy(su_nominal_force_world, su_matched_force_output_world);

  float yEta[3];
  float rCrossMatchedForce[3];
  vec3Cross(rCrossMatchedForce, contactOffsetWorld, su_matched_force_output_world);
  vec3Sub(yEta, rCrossMatchedForce, su_matched_torque_output_world);

  float rCrossForceRaw[3];
  vec3Cross(rCrossForceRaw, contactOffsetWorld, lumpedForceWorld);
  vec3Sub(su_point_contact_residual_world, rCrossForceRaw, lumpedTorqueWorld);

  float epsEta[3];
  float etaOffsetY[3];
  vec3Scale(etaOffsetY, yEta, su_eta_hat - SU_THRUST_EFFECTIVENESS_INIT);
  vec3Sub(epsEta, su_point_contact_residual_world, etaOffsetY);

  const float denominator = sanitizePositive(SU_THRUST_EFFECTIVENESS_RHO, 0.01f) + vec3Dot(yEta, yEta);
  su_eta_hat += dt * fmaxf(0.0f, SU_THRUST_EFFECTIVENESS_GAMMA) * vec3Dot(yEta, epsEta) / denominator;
  su_eta_hat = clampFinite(su_eta_hat);
  if (su_eta_hat < SU_THRUST_EFFECTIVENESS_EPS) {
    su_eta_hat = SU_THRUST_EFFECTIVENESS_EPS;
  }

  float etaCorrection[3];
  vec3Scale(etaCorrection, su_matched_force_output_world, su_eta_hat - SU_THRUST_EFFECTIVENESS_INIT);
  vec3Sub(su_corrected_force_world, lumpedForceWorld, etaCorrection);
  sanitizeVec3(su_corrected_force_world);
}

void suThrustEffectivenessGetMatchedForceWorld(float outF[3])
{
  if (!outF) {
    return;
  }
  vec3Copy(outF, su_nominal_force_world);
}

void suThrustEffectivenessGetPointContactResidualWorld(float outE[3])
{
  if (!outE) {
    return;
  }
  vec3Copy(outE, su_point_contact_residual_world);
}

void suThrustEffectivenessGetCorrectedForceWorld(float outF[3])
{
  if (!outF) {
    return;
  }
  vec3Copy(outF, su_corrected_force_world);
}

void suThrustEffectivenessGetEtaHat(float *outEta)
{
  if (!outEta) {
    return;
  }
  *outEta = su_eta_hat;
}

LOG_GROUP_START(suThrustEff)
LOG_ADD(LOG_FLOAT, etaHat, &su_eta_hat)
LOG_ADD(LOG_FLOAT, matchFx, &su_nominal_force_world[0])
LOG_ADD(LOG_FLOAT, matchFy, &su_nominal_force_world[1])
LOG_ADD(LOG_FLOAT, matchFz, &su_nominal_force_world[2])
LOG_ADD(LOG_FLOAT, epsTx, &su_point_contact_residual_world[0])
LOG_ADD(LOG_FLOAT, epsTy, &su_point_contact_residual_world[1])
LOG_ADD(LOG_FLOAT, epsTz, &su_point_contact_residual_world[2])
LOG_ADD(LOG_FLOAT, corrFx, &su_corrected_force_world[0])
LOG_ADD(LOG_FLOAT, corrFy, &su_corrected_force_world[1])
LOG_ADD(LOG_FLOAT, corrFz, &su_corrected_force_world[2])
LOG_GROUP_STOP(suThrustEff)
