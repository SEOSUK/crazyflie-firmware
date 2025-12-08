#pragma once

#include "stabilizer_types.h"
#include "motors.h"
#include "sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

void suWrenchObserverInit(void);

void suWrenchObserverUpdate(const state_t *state,
                            const motors_thrust_pwm_t *motorPwm,
                            const Axis3f *gyro_deg_s,
                            const float vW[3]);
                            
void suWrenchObserverGetWorldForce(float outF[3]);

#ifdef __cplusplus
}
#endif
