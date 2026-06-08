#include "su_position_reference.h"

#include <math.h>
#include <stdbool.h>

#include "log.h"
#include "su_params.h"
#include "su_position_trigger.h"
#include "su_trajectory_generator.h"
#include "su_wrench_observer.h"

#define SU_POSITION_VELOCITY_RATE_HZ 100
#define SU_RAD2DEG (180.0f / (float)M_PI)
#define SU_YAW_ALIGN_SAT_DEG 70.0f
#define SU_NORMAL_EST_EIG_ITERS 6
#define SU_NORMAL_PROJ_VEL_LPF_HZ 1.0f

static bool referenceInitialized = false;
static point_t referencePosition;
static float referenceBaseYawDeg = 0.0f;
static float referenceYawCorrectionDeg = 0.0f;
static uint8_t lastPositionMode = SU_POSITION_MODE_POSITION;
static uint8_t lastTrajectoryMode = SU_TRAJECTORY_NONE;
static uint8_t lastCommandReference = SU_COMMAND_REFERENCE_END_EFFECTOR;
static point_t trajectoryLocalOffsetPrev;
static bool trajectoryLocalOffsetInitialized = false;
static float referenceYawDegLog = 0.0f;
static float normalEstimatorMatrix[3][3];
static float normalForceEvidenceWorld[3] = {-1.0f, 0.0f, 0.0f};
static float normalProjectedCandidateWorld[3] = {-1.0f, 0.0f, 0.0f};
static float normalEstimateWorld[3] = {-1.0f, 0.0f, 0.0f};
static float filteredContactVelWorld[3] = {0.0f, 0.0f, 0.0f};
static float previousNormalEstimateWorld[3] = {-1.0f, 0.0f, 0.0f};
static float omegaNRaw = 0.0f;
static float omegaNLpf = 0.0f;
static float normalVelocityLeakageRaw = 0.0f;
static float normalVelocityLeakageLpf = 0.0f;
static float alphaFrameLog = 1.0f;
static float tangentialCmd1DesLog = 0.0f;
static float tangentialCmd2DesLog = 0.0f;
static bool filteredContactVelInitialized = false;
static bool normalEstimateInitialized = false;
static bool normalMetricsInitialized = false;

static float wrapAngleDeg180(const float angleDeg);

static float getReferenceYawDeg(void)
{
  return wrapAngleDeg180(referenceBaseYawDeg + referenceYawCorrectionDeg);
}

static bool isPositionSetpointCandidate(const setpoint_t *setpoint)
{
  if (!setpoint) {
    return false;
  }

  return setpoint->mode.x == modeAbs &&
         setpoint->mode.y == modeAbs &&
         setpoint->mode.z == modeAbs &&
         setpoint->mode.yaw == modeAbs &&
         setpoint->mode.roll == modeDisable &&
         setpoint->mode.pitch == modeDisable &&
         setpoint->mode.quat == modeDisable;
}

static void rotateBodyOffsetToWorld(const float yawDeg, point_t *offsetWorld)
{
  if (!offsetWorld) {
    return;
  }

  const float yawRad = yawDeg * ((float)M_PI / 180.0f);
  const float cosYaw = cosf(yawRad);
  const float sinYaw = sinf(yawRad);

  offsetWorld->x = cosYaw * su_r_offset_x - sinYaw * su_r_offset_y;
  offsetWorld->y = sinYaw * su_r_offset_x + cosYaw * su_r_offset_y;
  offsetWorld->z = su_r_offset_z;
}

static void convertReferencePosition(point_t *position, const uint8_t fromReference, const uint8_t toReference, const float yawDeg)
{
  if (!position || fromReference == toReference) {
    return;
  }

  point_t offsetWorld;
  rotateBodyOffsetToWorld(yawDeg, &offsetWorld);

  if (fromReference == SU_COMMAND_REFERENCE_DRONE && toReference == SU_COMMAND_REFERENCE_END_EFFECTOR) {
    position->x += offsetWorld.x;
    position->y += offsetWorld.y;
    position->z += offsetWorld.z;
  } else if (fromReference == SU_COMMAND_REFERENCE_END_EFFECTOR && toReference == SU_COMMAND_REFERENCE_DRONE) {
    position->x -= offsetWorld.x;
    position->y -= offsetWorld.y;
    position->z -= offsetWorld.z;
  }
}

static void initializeReference(const setpoint_t *setpoint, const state_t *state, const uint8_t commandReference)
{
  float initialYawDeg = 0.0f;

  if (setpoint && isPositionSetpointCandidate(setpoint)) {
    referencePosition = setpoint->position;
    initialYawDeg = setpoint->attitude.yaw;
  } else if (state) {
    referencePosition = state->position;
    initialYawDeg = state->attitude.yaw;
  } else {
    referencePosition.x = 0.0f;
    referencePosition.y = 0.0f;
    referencePosition.z = 0.0f;
    initialYawDeg = 0.0f;
  }

  referenceBaseYawDeg = wrapAngleDeg180(initialYawDeg);
  referenceYawCorrectionDeg = 0.0f;
  convertReferencePosition(&referencePosition, SU_COMMAND_REFERENCE_DRONE, commandReference, getReferenceYawDeg());
  referenceInitialized = true;
}

static void writeReferenceToSetpoint(setpoint_t *setpoint, const uint8_t commandReference)
{
  point_t droneReference = referencePosition;
  const float referenceYawDeg = getReferenceYawDeg();
  referenceYawDegLog = referenceYawDeg;

  convertReferencePosition(&droneReference, commandReference, SU_COMMAND_REFERENCE_DRONE, referenceYawDeg);

  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  setpoint->mode.yaw = modeAbs;

  setpoint->position = droneReference;
  setpoint->attitude.yaw = referenceYawDeg;
}

static float clampPositive(const float value)
{
  return (value > 0.0f) ? value : 0.0f;
}

static float wrapAngleDeg180(const float angleDeg)
{
  float wrapped = fmodf(angleDeg + 180.0f, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  return wrapped - 180.0f;
}

static float clampSymmetric(const float value, const float limit)
{
  const float positiveLimit = clampPositive(limit);
  if (value > positiveLimit) {
    return positiveLimit;
  }
  if (value < -positiveLimit) {
    return -positiveLimit;
  }
  return value;
}

static float vec3Dot(const float a[3], const float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

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

static void vec3Sub(float out[3], const float a[3], const float b[3])
{
  out[0] = a[0] - b[0];
  out[1] = a[1] - b[1];
  out[2] = a[2] - b[2];
}

static void vec3Cross(float out[3], const float a[3], const float b[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

static float vec3Norm(const float v[3])
{
  return sqrtf(vec3Dot(v, v));
}

static bool vec3Normalize(float out[3], const float in[3], const float eps)
{
  const float norm = vec3Norm(in);
  if (norm <= eps) {
    return false;
  }

  const float invNorm = 1.0f / norm;
  out[0] = in[0] * invNorm;
  out[1] = in[1] * invNorm;
  out[2] = in[2] * invNorm;
  return true;
}

static void getFixedNormalWorld(float outNormal[3])
{
  if (!outNormal) {
    return;
  }

  outNormal[0] = -1.0f;
  outNormal[1] = 0.0f;
  outNormal[2] = 0.0f;
}

static bool isNormalEstimatorEnabled(void)
{
  return su_normal_estimation != 0;
}

static bool isContactFrameControlEnabled(void)
{
  return isNormalEstimatorEnabled();
}

static void resetNormalEstimator(void)
{
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      normalEstimatorMatrix[row][col] = 0.0f;
    }
  }

  getFixedNormalWorld(normalEstimateWorld);
  getFixedNormalWorld(normalForceEvidenceWorld);
  getFixedNormalWorld(normalProjectedCandidateWorld);
  filteredContactVelWorld[0] = 0.0f;
  filteredContactVelWorld[1] = 0.0f;
  filteredContactVelWorld[2] = 0.0f;
  previousNormalEstimateWorld[0] = normalEstimateWorld[0];
  previousNormalEstimateWorld[1] = normalEstimateWorld[1];
  previousNormalEstimateWorld[2] = normalEstimateWorld[2];
  omegaNRaw = 0.0f;
  omegaNLpf = 0.0f;
  normalVelocityLeakageRaw = 0.0f;
  normalVelocityLeakageLpf = 0.0f;
  filteredContactVelInitialized = false;
  normalEstimateInitialized = false;
  normalMetricsInitialized = false;
}

static void getEstimatedNormalWorld(float outNormal[3])
{
  if (!outNormal) {
    return;
  }

  if (isNormalEstimatorEnabled() && normalEstimateInitialized) {
    vec3Copy(outNormal, normalEstimateWorld);
    return;
  }

  getFixedNormalWorld(outNormal);
}

static void getControlNormalWorld(float outNormal[3])
{
  if (!outNormal) {
    return;
  }

  if (isContactFrameControlEnabled()) {
    getEstimatedNormalWorld(outNormal);
  } else {
    getFixedNormalWorld(outNormal);
  }
}

static void computeDominantEigenvectorSymmetric3x3(float outVec[3], const float A[3][3], const float fallback[3])
{
  float iterVec[3];
  if (!vec3Normalize(iterVec, fallback, 1e-6f)) {
    getFixedNormalWorld(iterVec);
  }

  for (int iter = 0; iter < SU_NORMAL_EST_EIG_ITERS; ++iter) {
    float nextVec[3] = {
      A[0][0] * iterVec[0] + A[0][1] * iterVec[1] + A[0][2] * iterVec[2],
      A[1][0] * iterVec[0] + A[1][1] * iterVec[1] + A[1][2] * iterVec[2],
      A[2][0] * iterVec[0] + A[2][1] * iterVec[1] + A[2][2] * iterVec[2],
    };

    if (!vec3Normalize(iterVec, nextVec, 1e-9f)) {
      break;
    }
  }

  vec3Copy(outVec, iterVec);
}

static void updateNormalEstimator(void)
{
  if (!isNormalEstimatorEnabled()) {
    getFixedNormalWorld(normalEstimateWorld);
    normalEstimateInitialized = false;
    return;
  }

  float worldForce[3] = {0.0f, 0.0f, 0.0f};
  suWrenchObserverGetWorldForce(worldForce);

  const float epsilonF = clampPositive(su_normal_epsilon_f);
  if (vec3Norm(worldForce) <= epsilonF) {
    return;
  }

  float qf[3];
  if (!vec3Normalize(qf, worldForce, epsilonF)) {
    return;
  }
  vec3Copy(normalForceEvidenceWorld, qf);

  float contactVelWorld[3] = {0.0f, 0.0f, 0.0f};
  suWrenchObserverGetContactPointVelocityWorld(contactVelWorld);

  if (!filteredContactVelInitialized) {
    vec3Copy(filteredContactVelWorld, contactVelWorld);
    filteredContactVelInitialized = true;
  } else {
    const float dt = 1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ;
    const float cutoffHz = SU_NORMAL_PROJ_VEL_LPF_HZ;
    const float tau = 1.0f / (2.0f * (float)M_PI * cutoffHz);
    const float alpha = dt / (tau + dt);

    for (int i = 0; i < 3; ++i) {
      filteredContactVelWorld[i] += alpha * (contactVelWorld[i] - filteredContactVelWorld[i]);
    }
  }

  const float epsilonG = clampPositive(su_normal_epsilon_g);
  const float velNormSq = vec3Dot(filteredContactVelWorld, filteredContactVelWorld);
  const float velProjScale = vec3Dot(filteredContactVelWorld, qf) / (velNormSq + epsilonG);

  float qg[3];
  qg[0] = qf[0] - filteredContactVelWorld[0] * velProjScale;
  qg[1] = qf[1] - filteredContactVelWorld[1] * velProjScale;
  qg[2] = qf[2] - filteredContactVelWorld[2] * velProjScale;

  float nRaw[3];
  if (!vec3Normalize(nRaw, qg, 1e-6f)) {
    return;
  }
  vec3Copy(normalProjectedCandidateWorld, nRaw);

  float projectorRaw[3][3];
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      projectorRaw[row][col] = nRaw[row] * nRaw[col];
    }
  }

  const float dt = 1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ;
  const float beta = clampPositive(su_normal_beta);
  const float sigma = beta;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const float matrixDot = -beta * normalEstimatorMatrix[row][col] + sigma * projectorRaw[row][col];
      normalEstimatorMatrix[row][col] += dt * matrixDot;
    }
  }

  float candidate[3];
  const float *seed = normalEstimateInitialized ? normalEstimateWorld : nRaw;
  computeDominantEigenvectorSymmetric3x3(candidate, normalEstimatorMatrix, seed);
  if (vec3Dot(candidate, seed) < 0.0f) {
    vec3Scale(candidate, candidate, -1.0f);
  }

  vec3Copy(normalEstimateWorld, candidate);
  normalEstimateInitialized = true;

  const float cutoffHz = SU_NORMAL_PROJ_VEL_LPF_HZ;
  const float tau = 1.0f / (2.0f * (float)M_PI * cutoffHz);
  const float alpha = dt / (tau + dt);

  float normalDerivative[3] = {0.0f, 0.0f, 0.0f};
  if (normalMetricsInitialized) {
    vec3Sub(normalDerivative, normalEstimateWorld, previousNormalEstimateWorld);
    vec3Scale(normalDerivative, normalDerivative, 1.0f / dt);
  }
  omegaNRaw = vec3Norm(normalDerivative);

  const float velocityNorm = vec3Norm(filteredContactVelWorld);
  normalVelocityLeakageRaw = fabsf(
    vec3Dot(normalEstimateWorld, filteredContactVelWorld) / (velocityNorm + 1.0e-6f));

  if (!normalMetricsInitialized) {
    omegaNLpf = omegaNRaw;
    normalVelocityLeakageLpf = normalVelocityLeakageRaw;
    normalMetricsInitialized = true;
  } else {
    omegaNLpf += alpha * (omegaNRaw - omegaNLpf);
    normalVelocityLeakageLpf += alpha * (normalVelocityLeakageRaw - normalVelocityLeakageLpf);
  }
  vec3Copy(previousNormalEstimateWorld, normalEstimateWorld);
}

static void buildContactFrame(const float normalWorld[3], float t1World[3], float t2World[3])
{
  const float alphaRef[3] = {0.0f, 0.0f, 1.0f};
  float t1Candidate[3];
  vec3Cross(t1Candidate, alphaRef, normalWorld);

  if (!vec3Normalize(t1World, t1Candidate, 1e-6f)) {
    const float fallbackAxis[3] = {0.0f, 1.0f, 0.0f};
    vec3Cross(t1Candidate, fallbackAxis, normalWorld);
    if (!vec3Normalize(t1World, t1Candidate, 1e-6f)) {
      t1World[0] = 0.0f;
      t1World[1] = -1.0f;
      t1World[2] = 0.0f;
    }
  }

  vec3Cross(t2World, normalWorld, t1World);
  if (!vec3Normalize(t2World, t2World, 1e-6f)) {
    t2World[0] = 0.0f;
    t2World[1] = 0.0f;
    t2World[2] = 1.0f;
  }
}

static float computeAlphaFrame(void)
{
  const float alphaMin = fminf(fmaxf(su_alpha_frame_min, 0.0f), 1.0f);
  const float alphaBar = su_alpha_frame_bar;
  const float alphaRange = 1.0f - alphaMin;
  const float alphaBarAbs = fabsf(alphaBar);

  if (alphaRange <= 0.0f || alphaBarAbs <= 1.0e-6f) {
    return 1.0f;
  }

  const float metric = (alphaBar > 0.0f) ? omegaNLpf : normalVelocityLeakageLpf;
  const float ratio = metric / alphaBarAbs;
  return alphaMin + alphaRange / (1.0f + ratio * ratio);
}

static void applyTangentialVelocityControl(float velocityCmdWorld[3])
{
  if (!velocityCmdWorld) {
    return;
  }

  if (!isContactFrameControlEnabled()) {
    alphaFrameLog = 1.0f;
    tangentialCmd1DesLog = velocityCmdWorld[1];
    tangentialCmd2DesLog = velocityCmdWorld[2];
    return;
  }

  float normalWorld[3];
  getControlNormalWorld(normalWorld);

  float t1World[3];
  float t2World[3];
  buildContactFrame(normalWorld, t1World, t2World);

  const float normalCoeff = velocityCmdWorld[0];
  const float alphaFrame = computeAlphaFrame();
  const float tangentialCoeff1 = alphaFrame * velocityCmdWorld[1];
  const float tangentialCoeff2 = alphaFrame * velocityCmdWorld[2];
  alphaFrameLog = alphaFrame;
  tangentialCmd1DesLog = tangentialCoeff1;
  tangentialCmd2DesLog = tangentialCoeff2;

  float remappedVelocity[3];
  remappedVelocity[0] = -normalCoeff * normalWorld[0] +
                        tangentialCoeff1 * t1World[0] +
                        tangentialCoeff2 * t2World[0];
  remappedVelocity[1] = -normalCoeff * normalWorld[1] +
                        tangentialCoeff1 * t1World[1] +
                        tangentialCoeff2 * t2World[1];
  remappedVelocity[2] = -normalCoeff * normalWorld[2] +
                        tangentialCoeff1 * t1World[2] +
                        tangentialCoeff2 * t2World[2];

  vec3Copy(velocityCmdWorld, remappedVelocity);
}

static bool isPreloadVelocityControlActive(const uint8_t positionMode,
                                           const float forceDesired)
{
  return positionMode == SU_POSITION_MODE_VELOCITY &&
         fabsf(forceDesired) > 1e-6f;
}

static void applyPreloadVelocityControl(float velocityCmdWorld[3],
                                        const state_t *state,
                                        const float forceDesired)
{
  if (!velocityCmdWorld || !state) {
    return;
  }

  float normalWorld[3];
  getControlNormalWorld(normalWorld);

  float worldForce[3] = {0.0f, 0.0f, 0.0f};
  suWrenchObserverGetWorldForce(worldForce);

  const float f_n = vec3Dot(normalWorld, worldForce);
  const float stateVelocityWorld[3] = {
    state->velocity.x,
    state->velocity.y,
    state->velocity.z,
  };
  const float r_v = vec3Dot(normalWorld, stateVelocityWorld);
  const float nu_n = clampSymmetric(
    su_g_nf * (forceDesired - f_n) + su_g_nv * r_v,
    su_nu_n_bar);

  velocityCmdWorld[0] -= nu_n * normalWorld[0];
  velocityCmdWorld[1] -= nu_n * normalWorld[1];
  velocityCmdWorld[2] -= nu_n * normalWorld[2];
}

static void updateYawFromMobForce(void)
{
  if (!isNormalEstimatorEnabled()) {
    referenceYawCorrectionDeg = 0.0f;
    return;
  }

  float normalWorld[3];
  getEstimatedNormalWorld(normalWorld);
  const float targetDirXY[2] = {-normalWorld[0], -normalWorld[1]};

  const float targetDirNormXY = sqrtf(targetDirXY[0] * targetDirXY[0] +
                                      targetDirXY[1] * targetDirXY[1]);
  if (targetDirNormXY <= 1.0e-6f) {
    referenceYawCorrectionDeg = 0.0f;
    return;
  }

  const float targetYawDeg = atan2f(targetDirXY[1], targetDirXY[0]) * SU_RAD2DEG;
  const float yawErrorDeg = wrapAngleDeg180(targetYawDeg - referenceBaseYawDeg);
  float yawCorrectionDeg = yawErrorDeg;
  const float yawAlignMaxDeg = SU_YAW_ALIGN_SAT_DEG;

  if (yawAlignMaxDeg > 0.0f) {
    yawCorrectionDeg = clampSymmetric(yawCorrectionDeg, yawAlignMaxDeg);
  }

  referenceYawCorrectionDeg = yawCorrectionDeg;
}

void suPositionReferenceInit(void)
{
  referenceInitialized = false;
  referencePosition.x = 0.0f;
  referencePosition.y = 0.0f;
  referencePosition.z = 0.0f;
  referenceBaseYawDeg = 0.0f;
  referenceYawCorrectionDeg = 0.0f;
  referenceYawDegLog = 0.0f;
  alphaFrameLog = 1.0f;
  tangentialCmd1DesLog = 0.0f;
  tangentialCmd2DesLog = 0.0f;
  lastPositionMode = SU_POSITION_MODE_POSITION;
  lastTrajectoryMode = SU_TRAJECTORY_NONE;
  lastCommandReference = SU_COMMAND_REFERENCE_END_EFFECTOR;
  trajectoryLocalOffsetPrev.x = 0.0f;
  trajectoryLocalOffsetPrev.y = 0.0f;
  trajectoryLocalOffsetPrev.z = 0.0f;
  trajectoryLocalOffsetInitialized = false;
  resetNormalEstimator();

  suPositionTriggerInit();
  suTrajectoryGeneratorInit();
}

void suPositionReferenceUpdateSetpoint(setpoint_t *setpoint, const state_t *state, stabilizerStep_t stabilizerStep)
{
  suPositionTriggerUpdate();

  if (!setpoint || !isPositionSetpointCandidate(setpoint)) {
    return;
  }

  if (!referenceInitialized) {
    initializeReference(setpoint, state, suPositionTriggerGetCommandReference());
  }

  const uint8_t positionMode = suPositionTriggerGetMode();
  const uint8_t trajectoryMode = suPositionTriggerGetTrajectoryMode();
  const uint8_t commandReference = suPositionTriggerGetCommandReference();
  const float forceDesired = suPositionTriggerGetForceDesired();
  const float currentReferenceYawDeg = getReferenceYawDeg();
  if (commandReference != lastCommandReference) {
    convertReferencePosition(&referencePosition, lastCommandReference, commandReference, currentReferenceYawDeg);
  }

  if (positionMode == SU_POSITION_MODE_POSITION) {
    if (lastPositionMode == SU_POSITION_MODE_VELOCITY) {
      point_t commandFramePosition = referencePosition;
      setpoint->position = commandFramePosition;
      setpoint->attitude.yaw = currentReferenceYawDeg;
    }

    referencePosition = setpoint->position;
    referenceBaseYawDeg = wrapAngleDeg180(setpoint->attitude.yaw);
    referenceYawCorrectionDeg = 0.0f;
    suTrajectoryGeneratorDeactivate();
    writeReferenceToSetpoint(setpoint, commandReference);
  } else {
    referenceBaseYawDeg = wrapAngleDeg180(setpoint->attitude.yaw);
    const float velocityReferenceYawDeg = getReferenceYawDeg();

    if (lastPositionMode == SU_POSITION_MODE_POSITION) {
      trajectoryLocalOffsetInitialized = false;
      if (trajectoryMode != SU_TRAJECTORY_NONE) {
        suTrajectoryGeneratorStart(trajectoryMode, &referencePosition, velocityReferenceYawDeg);
      } else {
        suTrajectoryGeneratorDeactivate();
      }
    }

    if (trajectoryMode != lastTrajectoryMode) {
      trajectoryLocalOffsetInitialized = false;
      if (trajectoryMode == SU_TRAJECTORY_NONE) {
        suTrajectoryGeneratorDeactivate();
      } else {
        suTrajectoryGeneratorStart(trajectoryMode, &referencePosition, velocityReferenceYawDeg);
      }
    }

    if (trajectoryMode == SU_TRAJECTORY_NONE) {
      suTrajectoryGeneratorDeactivate();
      if (RATE_DO_EXECUTE(SU_POSITION_VELOCITY_RATE_HZ, stabilizerStep)) {
        updateNormalEstimator();
        float velocityCmdWorld[3] = {
          setpoint->position.x,
          setpoint->position.y,
          setpoint->position.z,
        };
        applyTangentialVelocityControl(velocityCmdWorld);
        if (isPreloadVelocityControlActive(positionMode, forceDesired)) {
          applyPreloadVelocityControl(velocityCmdWorld, state, forceDesired);
        }
        referencePosition.x += velocityCmdWorld[0] * (1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ);
        referencePosition.y += velocityCmdWorld[1] * (1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ);
        referencePosition.z += velocityCmdWorld[2] * (1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ);
        updateYawFromMobForce();
      }
    } else {
      if (RATE_DO_EXECUTE(SU_POSITION_VELOCITY_RATE_HZ, stabilizerStep)) {
        updateNormalEstimator();
        point_t trajectoryLocalOffset = {0.0f, 0.0f, 0.0f};
        float trajectoryYawDeg = velocityReferenceYawDeg;
        suTrajectoryGeneratorUpdateLocalOffset(
          trajectoryMode, stabilizerStep, &trajectoryLocalOffset, &trajectoryYawDeg);

        float velocityCmdWorld[3] = {
          0.0f,
          0.0f,
          0.0f,
        };

        if (trajectoryLocalOffsetInitialized) {
          const float dt = 1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ;
          velocityCmdWorld[1] = (trajectoryLocalOffset.y - trajectoryLocalOffsetPrev.y) / dt;
          velocityCmdWorld[2] = (trajectoryLocalOffset.z - trajectoryLocalOffsetPrev.z) / dt;
        }
        trajectoryLocalOffsetPrev = trajectoryLocalOffset;
        trajectoryLocalOffsetInitialized = true;

        applyTangentialVelocityControl(velocityCmdWorld);
        if (isPreloadVelocityControlActive(positionMode, forceDesired)) {
          applyPreloadVelocityControl(velocityCmdWorld, state, forceDesired);
        }

        referencePosition.x += velocityCmdWorld[0] * (1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ);
        referencePosition.y += velocityCmdWorld[1] * (1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ);
        referencePosition.z += velocityCmdWorld[2] * (1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ);
        referenceBaseYawDeg = wrapAngleDeg180(trajectoryYawDeg);
      }
      if (RATE_DO_EXECUTE(SU_POSITION_VELOCITY_RATE_HZ, stabilizerStep)) {
        updateYawFromMobForce();
      }
    }

    writeReferenceToSetpoint(setpoint, commandReference);
  }

  lastPositionMode = positionMode;
  lastTrajectoryMode = trajectoryMode;
  lastCommandReference = commandReference;
}

LOG_GROUP_START(suPosRef)
LOG_ADD(LOG_FLOAT, nPreX, &normalForceEvidenceWorld[0])
LOG_ADD(LOG_FLOAT, nPreY, &normalForceEvidenceWorld[1])
LOG_ADD(LOG_FLOAT, nPreZ, &normalForceEvidenceWorld[2])
LOG_ADD(LOG_FLOAT, nPostX, &normalProjectedCandidateWorld[0])
LOG_ADD(LOG_FLOAT, nPostY, &normalProjectedCandidateWorld[1])
LOG_ADD(LOG_FLOAT, nPostZ, &normalProjectedCandidateWorld[2])
LOG_ADD(LOG_FLOAT, nEstX, &normalEstimateWorld[0])
LOG_ADD(LOG_FLOAT, nEstY, &normalEstimateWorld[1])
LOG_ADD(LOG_FLOAT, nEstZ, &normalEstimateWorld[2])
LOG_ADD(LOG_FLOAT, vEeX, &filteredContactVelWorld[0])
LOG_ADD(LOG_FLOAT, vEeY, &filteredContactVelWorld[1])
LOG_ADD(LOG_FLOAT, vEeZ, &filteredContactVelWorld[2])
LOG_ADD(LOG_FLOAT, omgN, &omegaNLpf)
LOG_ADD(LOG_FLOAT, nVelLeak, &normalVelocityLeakageLpf)
LOG_ADD(LOG_FLOAT, alphaFrm, &alphaFrameLog)
LOG_ADD(LOG_FLOAT, t1CmdDes, &tangentialCmd1DesLog)
LOG_ADD(LOG_FLOAT, t2CmdDes, &tangentialCmd2DesLog)
LOG_GROUP_STOP(suPosRef)
