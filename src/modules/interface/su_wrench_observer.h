#pragma once

#include "stabilizer_types.h"
#include "motors.h"
#include "sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

void suWrenchObserverInit(void);

void suWrenchObserverUpdate(const state_t *state,
                            const motors_thrust_uncapped_t *motorThrustUncapped,
                            const motors_thrust_pwm_t *motorPwm,
                            const Axis3f *gyro_deg_s,
                            const float vel_from_pos_world[3],
                            float dt);
                            
void suWrenchObserverGetWorldForce(float outF[3]);
void suWrenchObserverGetStateVelocityWorld(float outV[3]);
void suWrenchObserverGetContactPointVelocityWorld(float outV[3]);
void suWrenchObserverRequestZeroBias(void);

#ifdef __cplusplus
}
#endif
