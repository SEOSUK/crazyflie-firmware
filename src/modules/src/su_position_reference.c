#include "su_position_reference.h"

#include <math.h>
#include <stdbool.h>

#include "su_params.h"
#include "su_position_trigger.h"
#include "su_trajectory_generator.h"
#include "su_wrench_observer.h"

#define SU_POSITION_VELOCITY_RATE_HZ 100
#define SU_RAD2DEG (180.0f / (float)M_PI)

static bool referenceInitialized = false;
static point_t referencePosition;
static float referenceYawDeg = 0.0f;
static uint8_t lastPositionMode = SU_POSITION_MODE_POSITION;
static uint8_t lastTrajectoryMode = SU_TRAJECTORY_NONE;
static uint8_t lastCommandReference = SU_COMMAND_REFERENCE_END_EFFECTOR;
static float filteredForceWorldXY[2] = {0.0f, 0.0f};
static bool filteredForceInitialized = false;

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
  if (setpoint && isPositionSetpointCandidate(setpoint)) {
    referencePosition = setpoint->position;
    referenceYawDeg = setpoint->attitude.yaw;
  } else if (state) {
    referencePosition = state->position;
    referenceYawDeg = state->attitude.yaw;
  } else {
    referencePosition.x = 0.0f;
    referencePosition.y = 0.0f;
    referencePosition.z = 0.0f;
    referenceYawDeg = 0.0f;
  }

  convertReferencePosition(&referencePosition, SU_COMMAND_REFERENCE_DRONE, commandReference, referenceYawDeg);
  referenceInitialized = true;
}

static void writeReferenceToSetpoint(setpoint_t *setpoint, const uint8_t commandReference)
{
  point_t droneReference = referencePosition;

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

static float clampUnit(const float value)
{
  if (value <= 0.0f) {
    return 0.0f;
  }
  if (value >= 1.0f) {
    return 1.0f;
  }
  return value;
}

static float smoothstep01(const float value)
{
  const float t = clampUnit(value);
  return t * t * (3.0f - 2.0f * t);
}

static float wrapAngleDeg180(const float angleDeg)
{
  float wrapped = fmodf(angleDeg + 180.0f, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  return wrapped - 180.0f;
}

static void resetFilteredForce(void)
{
  filteredForceWorldXY[0] = 0.0f;
  filteredForceWorldXY[1] = 0.0f;
  filteredForceInitialized = false;
}

static void updateYawFromMobForce(void)
{
  if (clampPositive(su_yaw_force_lpf_hz) <= 0.0f) {
    return;
  }

  float worldForce[3] = {0.0f, 0.0f, 0.0f};
  suWrenchObserverGetWorldForce(worldForce);

  const float rawFx = worldForce[0];
  const float rawFy = worldForce[1];

  if (!filteredForceInitialized) {
    filteredForceWorldXY[0] = rawFx;
    filteredForceWorldXY[1] = rawFy;
    filteredForceInitialized = true;
  } else {
    const float dt = 1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ;
    const float cutoffHz = clampPositive(su_yaw_force_lpf_hz);
    const float tau = 1.0f / (2.0f * (float)M_PI * cutoffHz);
    const float alpha = dt / (tau + dt);

    filteredForceWorldXY[0] += alpha * (rawFx - filteredForceWorldXY[0]);
    filteredForceWorldXY[1] += alpha * (rawFy - filteredForceWorldXY[1]);
  }

  const float forceNormXY = sqrtf(filteredForceWorldXY[0] * filteredForceWorldXY[0] +
                                  filteredForceWorldXY[1] * filteredForceWorldXY[1]);
  const float targetYawDeg = atan2f(-filteredForceWorldXY[1], -filteredForceWorldXY[0]) * SU_RAD2DEG;
  const float epsilonFMin = clampPositive(su_epsilon_f_min);
  float epsilonFMax = clampPositive(su_epsilon_f_max);
  if (epsilonFMax < epsilonFMin) {
    epsilonFMax = epsilonFMin;
  }

  float yawAlignWeight = 1.0f;
  if (epsilonFMax > epsilonFMin) {
    yawAlignWeight = smoothstep01((forceNormXY - epsilonFMin) / (epsilonFMax - epsilonFMin));
  } else if (epsilonFMax > 0.0f) {
    yawAlignWeight = smoothstep01(forceNormXY / epsilonFMax);
  }

  if (yawAlignWeight > 0.0f) {
    const float yawErrorDeg = wrapAngleDeg180(targetYawDeg - referenceYawDeg);
    referenceYawDeg += yawAlignWeight * yawErrorDeg;
    referenceYawDeg = wrapAngleDeg180(referenceYawDeg);
  }
}

void suPositionReferenceInit(void)
{
  referenceInitialized = false;
  referencePosition.x = 0.0f;
  referencePosition.y = 0.0f;
  referencePosition.z = 0.0f;
  referenceYawDeg = 0.0f;
  lastPositionMode = SU_POSITION_MODE_POSITION;
  lastTrajectoryMode = SU_TRAJECTORY_NONE;
  lastCommandReference = SU_COMMAND_REFERENCE_END_EFFECTOR;
  resetFilteredForce();

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

  if (commandReference != lastCommandReference) {
    convertReferencePosition(&referencePosition, lastCommandReference, commandReference, referenceYawDeg);
  }

  if (positionMode == SU_POSITION_MODE_POSITION) {
    if (lastPositionMode == SU_POSITION_MODE_VELOCITY) {
      point_t commandFramePosition = referencePosition;
      setpoint->position = commandFramePosition;
      setpoint->attitude.yaw = referenceYawDeg;
    }

    referencePosition = setpoint->position;
    referenceYawDeg = setpoint->attitude.yaw;
    suTrajectoryGeneratorDeactivate();
    resetFilteredForce();
    writeReferenceToSetpoint(setpoint, commandReference);
  } else {
    if (lastPositionMode == SU_POSITION_MODE_POSITION) {
      resetFilteredForce();
      if (trajectoryMode != SU_TRAJECTORY_NONE) {
        suTrajectoryGeneratorStart(trajectoryMode, &referencePosition, referenceYawDeg);
      } else {
        suTrajectoryGeneratorDeactivate();
      }
    }

    if (trajectoryMode != lastTrajectoryMode) {
      if (trajectoryMode == SU_TRAJECTORY_NONE) {
        suTrajectoryGeneratorDeactivate();
      } else {
        suTrajectoryGeneratorStart(trajectoryMode, &referencePosition, referenceYawDeg);
      }
    }

    if (trajectoryMode == SU_TRAJECTORY_NONE) {
      suTrajectoryGeneratorDeactivate();
      if (RATE_DO_EXECUTE(SU_POSITION_VELOCITY_RATE_HZ, stabilizerStep)) {
        referencePosition.x += setpoint->position.x * (1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ);
        referencePosition.y += setpoint->position.y * (1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ);
        referencePosition.z += setpoint->position.z * (1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ);
        updateYawFromMobForce();
      }
    } else {
      if (!suTrajectoryGeneratorIsActive()) {
        suTrajectoryGeneratorStart(trajectoryMode, &referencePosition, referenceYawDeg);
      }
      suTrajectoryGeneratorUpdate(trajectoryMode, stabilizerStep, &referencePosition, &referenceYawDeg);
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
