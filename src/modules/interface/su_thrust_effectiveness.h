#pragma once

#include "stabilizer_types.h"
#include "motors.h"

#ifdef __cplusplus
extern "C" {
#endif

void suThrustEffectivenessInit(void);
void suThrustEffectivenessUpdate(const state_t *state,
                                const motors_thrust_uncapped_t *motorThrustReq,
                                const Axis3f *gyro_deg_s,
                                const float vel_from_pos_world[3],
                                float dt);

void suThrustEffectivenessGetMatchedForceWorld(float outF[3]);
void suThrustEffectivenessGetPointContactResidualWorld(float outE[3]);
void suThrustEffectivenessGetCorrectedForceWorld(float outF[3]);
void suThrustEffectivenessGetEtaHat(float *outEta);

#ifdef __cplusplus
}
#endif
