#include "su_position_reference.h"

#include <stdbool.h>

#include "su_position_trigger.h"
#include "su_trajectory_generator.h"

#define SU_POSITION_VELOCITY_RATE_HZ 100

static bool referenceInitialized = false;
static point_t referencePosition;
static float referenceYawDeg = 0.0f;
static uint8_t lastPositionMode = SU_POSITION_MODE_POSITION;
static uint8_t lastTrajectoryMode = SU_TRAJECTORY_NONE;

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

static void initializeReference(const setpoint_t *setpoint, const state_t *state)
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

  referenceInitialized = true;
}

static void writeReferenceToSetpoint(setpoint_t *setpoint)
{
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  setpoint->mode.yaw = modeAbs;

  setpoint->position = referencePosition;
  setpoint->attitude.yaw = referenceYawDeg;
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
    initializeReference(setpoint, state);
  }

  const uint8_t positionMode = suPositionTriggerGetMode();
  const uint8_t trajectoryMode = suPositionTriggerGetTrajectoryMode();

  if (positionMode == SU_POSITION_MODE_POSITION) {
    if (lastPositionMode == SU_POSITION_MODE_VELOCITY) {
      setpoint->position = referencePosition;
      setpoint->attitude.yaw = referenceYawDeg;
    }

    referencePosition = setpoint->position;
    referenceYawDeg = setpoint->attitude.yaw;
    suTrajectoryGeneratorDeactivate();
    writeReferenceToSetpoint(setpoint);
  } else {
    if (lastPositionMode == SU_POSITION_MODE_POSITION) {
      referencePosition = setpoint->position;
      referenceYawDeg = setpoint->attitude.yaw;
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
      }
      referenceYawDeg = setpoint->attitude.yaw;
    } else {
      if (!suTrajectoryGeneratorIsActive()) {
        suTrajectoryGeneratorStart(trajectoryMode, &referencePosition, referenceYawDeg);
      }
      suTrajectoryGeneratorUpdate(trajectoryMode, stabilizerStep, &referencePosition, &referenceYawDeg);
    }

    writeReferenceToSetpoint(setpoint);
  }

  lastPositionMode = positionMode;
  lastTrajectoryMode = trajectoryMode;
}
