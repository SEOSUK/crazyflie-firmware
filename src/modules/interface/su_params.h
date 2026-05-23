#ifndef SU_PARAMS_H_
#define SU_PARAMS_H_
/*
 * Centralized shared parameters for SU modules
 *
 * These are defined (storage allocated) in su_params.c and registered
 * to the Crazyflie PARAM system there. Include this header from any
 * module (estimator / controller / trajectory / observers ...) that
 * needs to read/write the same parameters at runtime.
 *
 * Units:
 *  - mass:            [kg]
 *  - Kf, Ktau:        [1/s]
 *  - deadzone_F:      [N]
 *  - deadzone_T:      [N·m]
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// -------- Platform / common --------
extern float su_mass;             // [kg]
extern float Jxx;                 // [kg·m^2]
extern float Jyy;                 // [kg·m^2]
extern float Jzz;                 // [kg·m^2]

// -------- Wrench observer / MOB --------
extern float su_Kf;               // [1/s] linear momentum observer gain
extern float su_Ktau;             // [1/s] angular momentum observer gain
extern float su_Kp;               // [1/s] translational momentum correction gain
extern float su_Kh;               // [1/s] rotational momentum correction gain
extern float su_Keps;             // [1/s] consistency residual correction gain

extern float su_deadzone_F;       // [N]   force deadzone
extern float su_deadzone_T;       // [N·m] torque deadzone

extern uint8_t su_zero_bias;      // [0/1] trigger MOB output bias capture
extern float su_com_offset_x;     // [m] body-frame CoM offset x
extern float su_com_offset_y;     // [m] body-frame CoM offset y
extern float su_com_offset_z;     // [m] body-frame CoM offset z
extern float su_r_offset_x;       // [m] body-frame contact offset x
extern float su_r_offset_y;       // [m] body-frame contact offset y
extern float su_r_offset_z;       // [m] body-frame contact offset z
extern uint8_t su_consistency_mode; // 0=None, 1=Residual, 2=Both

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* SU_PARAMS_H_ */
