#include "su_position_trigger.h"

#include "app_channel.h"

#define SU_POSITION_TRIGGER_MAGIC   0xA5
#define SU_POSITION_TRIGGER_VERSION 0x01

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t version;
  uint8_t positionMode;
  uint8_t trajectoryMode;
  uint8_t commandReference;
} su_position_trigger_packet_t;

static uint8_t currentPositionMode = SU_POSITION_MODE_POSITION;
static uint8_t currentTrajectoryMode = SU_TRAJECTORY_NONE;
static uint8_t currentCommandReference = SU_COMMAND_REFERENCE_END_EFFECTOR;

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
}

void suPositionTriggerUpdate(void)
{
  su_position_trigger_packet_t packet;

  while (appchannelReceiveDataPacket(&packet, sizeof(packet), 0) >= sizeof(packet)) {
    if (packet.magic != SU_POSITION_TRIGGER_MAGIC || packet.version != SU_POSITION_TRIGGER_VERSION) {
      continue;
    }

    currentPositionMode = sanitizePositionMode(packet.positionMode);
    currentTrajectoryMode = sanitizeTrajectoryMode(packet.trajectoryMode);
    currentCommandReference = sanitizeCommandReference(packet.commandReference);
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
