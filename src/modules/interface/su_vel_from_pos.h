// su_vel_from_pos.h
#pragma once

#include "estimator.h"  // state_t

#ifdef __cplusplus
extern "C" {
#endif

void suVelFromPosInit(void);

// dt: [s], 호출 주파수는 1 kHz (stabilizer loop)
void suVelFromPosUpdate(const state_t *state, float dt);

// World-frame velocity [m/s]
void suVelFromPosGetWorld(float out_vW[3]);

#ifdef __cplusplus
}
#endif
