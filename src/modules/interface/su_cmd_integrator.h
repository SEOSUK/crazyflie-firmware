#pragma once

#include "stabilizer_types.h"  // setpoint_t, state_t 정의돼 있는 헤더

void suCmdIntegratorInit(void);
void suCmdIntegratorUpdate(float dt,
                           const setpoint_t* sp_in,
                           const state_t* state,
                           const float* f_ext_world,   // ★ 추가 (길이 3짜리, [Fx,Fy,Fz])
                           setpoint_t* sp_out);