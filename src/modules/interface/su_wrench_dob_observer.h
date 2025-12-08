#pragma once

#include "stabilizer_types.h"
#include "sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

void suWrenchObserverDOBInit(void);

void suWrenchObserverDOBUpdate(const state_t *state,
                               const motors_thrust_pwm_t *motorPwm,
                               const Axis3f *gyro_deg_s,
                               const float vW[3]);
                               
void suWrenchObserverDOBGetWorldForce(float outF[3]);

#ifdef __cplusplus
}
#endif
