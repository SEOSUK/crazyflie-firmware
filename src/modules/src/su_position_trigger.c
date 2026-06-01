#include "su_position_trigger.h"

#include "app_channel.h"
#include "su_params.h"
#include <math.h>

#define SU_POSITION_TRIGGER_MAGIC   0xA5
#define SU_POSITION_TRIGGER_VERSION 0x02
#define SU_HOVER_CALIBRATION_TRIGGER_MAGIC   0xA6
#define SU_HOVER_CALIBRATION_TRIGGER_VERSION 0x01

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t version;
  uint8_t positionMode;
  uint8_t trajectoryMode;
  uint8_t commandReference;
  float forceDesired;
} su_position_trigger_packet_t;

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t version;
  float mass;
  float comOffX;
  float comOffY;
} su_hover_calibration_packet_t;

static uint8_t currentPositionMode = SU_POSITION_MODE_POSITION;
static uint8_t currentTrajectoryMode = SU_TRAJECTORY_NONE;
static uint8_t currentCommandReference = SU_COMMAND_REFERENCE_END_EFFECTOR;
static float currentForceDesired = 0.0f;

static uint8_t sanitizePositionMode(const uint8_t mode)
{
  return (mode == SU_POSITION_MODE_VELOCITY) ? SU_POSITION_MODE_VELOCITY : SU_POSITION_MODE_POSITION;
}

static uint8_t sanitizeTrajectoryMode(const uint8_t mode)
{
  return (mode <= SU_TRAJECTORY_2) ? mode : SU_TRAJECTORY_NONE;
}

static uint8_t sanitizeCommandReference(const uint8_t reference)
{
  return (reference == SU_COMMAND_REFERENCE_END_EFFECTOR) ?
    SU_COMMAND_REFERENCE_END_EFFECTOR : SU_COMMAND_REFERENCE_DRONE;
}

void suPositionTriggerInit(void)
{
  currentPositionMode = SU_POSITION_MODE_POSITION;
  currentTrajectoryMode = SU_TRAJECTORY_NONE;
  currentCommandReference = SU_COMMAND_REFERENCE_END_EFFECTOR;
  currentForceDesired = 0.0f;
}

void suPositionTriggerUpdate(void)
{
  uint8_t packetBuffer[APPCHANNEL_MTU];
  size_t packetLength = 0;

  while ((packetLength = appchannelReceiveDataPacket(packetBuffer, sizeof(packetBuffer), 0)) > 0) {
    if (packetLength >= sizeof(su_position_trigger_packet_t)) {
      const su_position_trigger_packet_t* packet = (const su_position_trigger_packet_t*)packetBuffer;
      if (packet->magic == SU_POSITION_TRIGGER_MAGIC && packet->version == SU_POSITION_TRIGGER_VERSION) {
        currentPositionMode = sanitizePositionMode(packet->positionMode);
        currentTrajectoryMode = sanitizeTrajectoryMode(packet->trajectoryMode);
        currentCommandReference = sanitizeCommandReference(packet->commandReference);
        currentForceDesired = isfinite(packet->forceDesired) ? packet->forceDesired : 0.0f;
        continue;
      }
    }

    if (packetLength >= sizeof(su_hover_calibration_packet_t)) {
      const su_hover_calibration_packet_t* packet = (const su_hover_calibration_packet_t*)packetBuffer;
      if (packet->magic == SU_HOVER_CALIBRATION_TRIGGER_MAGIC &&
          packet->version == SU_HOVER_CALIBRATION_TRIGGER_VERSION) {
        if (isfinite(packet->mass) && packet->mass > 0.0f) {
          su_mass = packet->mass;
        }
        if (isfinite(packet->comOffX)) {
          su_com_offset_x = packet->comOffX;
        }
        if (isfinite(packet->comOffY)) {
          su_com_offset_y = packet->comOffY;
        }
      }
    }
  }
}

uint8_t suPositionTriggerGetMode(void)
{
  return currentPositionMode;
}

uint8_t suPositionTriggerGetTrajectoryMode(void)
{
  return currentTrajectoryMode;
}

uint8_t suPositionTriggerGetCommandReference(void)
{
  return currentCommandReference;
}

float suPositionTriggerGetForceDesired(void)
{
  return currentForceDesired;
}
