#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void suTrajectoryGeneratorInit(void);
void suTrajectoryGeneratorDeactivate(void);
void suTrajectoryGeneratorStart(uint8_t trajectoryMode, const point_t *origin, float yawDeg);
bool suTrajectoryGeneratorIsActive(void);
void suTrajectoryGeneratorUpdate(uint8_t trajectoryMode, stabilizerStep_t stabilizerStep, point_t *position, float *yawDeg);

#ifdef __cplusplus
}
#endif
