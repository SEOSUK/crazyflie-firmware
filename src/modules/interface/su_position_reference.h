#pragma once

#include "stabilizer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void suPositionReferenceInit(void);
void suPositionReferenceUpdateSetpoint(setpoint_t *setpoint, const state_t *state, stabilizerStep_t stabilizerStep);

#ifdef __cplusplus
}
#endif
