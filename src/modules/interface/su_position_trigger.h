#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  SU_POSITION_MODE_POSITION = 0,
  SU_POSITION_MODE_VELOCITY = 1,
} su_position_mode_t;

typedef enum {
  SU_TRAJECTORY_NONE = 0,
  SU_TRAJECTORY_1 = 1,
  SU_TRAJECTORY_2 = 2,
} su_trajectory_mode_t;

void suPositionTriggerInit(void);
void suPositionTriggerUpdate(void);

uint8_t suPositionTriggerGetMode(void);
uint8_t suPositionTriggerGetTrajectoryMode(void);

#ifdef __cplusplus
}
#endif
