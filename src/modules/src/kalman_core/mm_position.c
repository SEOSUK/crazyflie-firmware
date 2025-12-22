/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "mm_position.h"
#include "kalman_core.h"

void kalmanCoreUpdateWithPosition(kalmanCoreData_t* this, positionMeasurement_t *xyz)
{
  // direct measurement of x,y,z
  for (int i = 0; i < 3; i++) {
    float h[KC_STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_X + i] = 1;

    // Mask out attitude error states (D0/D1/D2) so external position does not "tilt" attitude.
    kalmanCoreScalarUpdateMasked(
      this,
      &H,
      xyz->pos[i] - this->S[KC_STATE_X + i],
      xyz->stdDev,
      (KC_UPD_POS | KC_UPD_VEL)   // <-- 추천
      // (KC_UPD_POS)             // <-- position만 업데이트 하고 싶으면 이걸로
    );
  }
}