/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
 * \verbatim
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 * \endverbatim
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from OS related functionality
 */

#include "kalman_core.h"
#include "kalman_core_params_defaults.h"
#include "cfassert.h"
#include "autoconf.h"

#include "physicalConstants.h"

#include "math3d.h"
#include "static_mem.h"

// #define DEBUG_STATE_CHECK

/**
 * Supporting and utility functions
 */

#ifdef DEBUG_STATE_CHECK
static void assertStateNotNaN(const kalmanCoreData_t* this) {
  if ((isnan(this->S[KC_STATE_X])) ||
      (isnan(this->S[KC_STATE_Y])) ||
      (isnan(this->S[KC_STATE_Z])) ||
      (isnan(this->S[KC_STATE_PX])) ||
      (isnan(this->S[KC_STATE_PY])) ||
      (isnan(this->S[KC_STATE_PZ])) ||
      (isnan(this->S[KC_STATE_D0])) ||
      (isnan(this->S[KC_STATE_D1])) ||
      (isnan(this->S[KC_STATE_D2])) ||
      (isnan(this->q[0])) ||
      (isnan(this->q[1])) ||
      (isnan(this->q[2])) ||
      (isnan(this->q[3])))
  {
    ASSERT(false);
  }

  for(int i=0; i<KC_STATE_DIM; i++) {
    for(int j=0; j<KC_STATE_DIM; j++)
    {
      if (isnan(this->P[i][j]))
      {
        ASSERT(false);
      }
    }
  }
}
#else
static void assertStateNotNaN(const kalmanCoreData_t* this)
{
  return;
}
#endif



// SEUK
static void quatEnsurePositiveWLocal(float q[4])
{
  if (q[0] < 0.0f) {
    q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3];
  }
}

// -------------------- Complementary attitude filter (quaternion) --------------------

#define COMP_EPS          (1e-6f)
#define COMP_ACC_MIN_NORM (9.0f)
#define COMP_ACC_MAX_NORM (11.f)

// q 정규화
static void quatNormalize(float q[4])
{
  float n = arm_sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]) + COMP_EPS;
  q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
}

// -------------------- (A) Complementary attitude -> Kalman fusion --------------------

static void quatConj(const float q[4], float qc[4])
{
  qc[0] = q[0];
  qc[1] = -q[1];
  qc[2] = -q[2];
  qc[3] = -q[3];
}

static void quatMul(const float a[4], const float b[4], float out[4])
{
  out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

// yaw(q) [rad] (same convention as your Euler extraction)
static float quatYawRad(const float q[4])
{
  const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  return atan2f(2.0f*(qx*qy + qw*qz), (qw*qw + qx*qx - qy*qy - qz*qz));
}

static void quatFromYawRad(float yaw, float qYaw[4])
{
  const float half = 0.5f * yaw;
  qYaw[0] = arm_cos_f32(half);
  qYaw[1] = 0.0f;
  qYaw[2] = 0.0f;
  qYaw[3] = arm_sin_f32(half);
  quatNormalize(qYaw);
}

// qOut = qYaw(kalman) ⊗ qTilt(comp),  where qTilt(comp) = conj(qYaw(comp)) ⊗ qComp
static void buildMixedQuatYawKalmanTiltComp(const float qKal[4], const float qComp[4], float qOut[4])
{
  // 1) yaw-only quats
  float qYawKal[4], qYawComp[4];
  quatFromYawRad(quatYawRad(qKal),  qYawKal);
  quatFromYawRad(quatYawRad(qComp), qYawComp);

  // 2) qTiltComp = conj(qYawComp) ⊗ qComp
  float qYawCompConj[4];
  quatConj(qYawComp, qYawCompConj);

  float qTiltComp[4];
  quatMul(qYawCompConj, qComp, qTiltComp);
  quatNormalize(qTiltComp);
  quatEnsurePositiveWLocal(qTiltComp);

  // 3) qOut = qYawKal ⊗ qTiltComp
  quatMul(qYawKal, qTiltComp, qOut);
  quatNormalize(qOut);

  quatEnsurePositiveWLocal(qOut);
}

static void fuseCompRollPitchToKalman(kalmanCoreData_t* this, float std_rad)
{
  // Measurement model: z = D0, z = D1
  // z comes from qComp ⊗ q^{-1} small-angle error
  if (std_rad < 1e-6f) {
    std_rad = 1e-3f;
  }

  // q_err = qComp ⊗ conj(q)
  float q_conj[4];
  float q_err[4];
  quatConj(this->q, q_conj);
  quatMul(this->qComp, q_conj, q_err);
  quatNormalize(q_err);
  quatEnsurePositiveWLocal(q_err);

  // small-angle approx: v ≈ 2 * vec(q_err)
  const float v0 = 2.0f * q_err[1]; // roll-ish error
  const float v1 = 2.0f * q_err[2]; // pitch-ish error
  // const float v2 = 2.0f * q_err[3]; // yaw error (ignored)

  // Fuse D0
  {
    float h[KC_STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_D0] = 1.0f;

    // innovation: z - D0
    const float innov = v0 - this->S[KC_STATE_D0];
    kalmanCoreScalarUpdate(this, &H, innov, std_rad);
  }

  // Fuse D1
  {
    float h[KC_STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_D1] = 1.0f;

    const float innov = v1 - this->S[KC_STATE_D1];
    kalmanCoreScalarUpdate(this, &H, innov, std_rad);
  }
}

void kalmanCoreSetFuseComplementaryToKalman(kalmanCoreData_t* this, bool enable)
{
  this->fuseComplementaryToKalman = enable;
}

void kalmanCoreSetCompFuseStdRP(kalmanCoreData_t* this, float std_rad)
{
  if (std_rad < 0.0f) std_rad = 0.0f;
  if (std_rad > 1.0f) std_rad = 1.0f; // rad 단위, 필요시 더 늘려도 됨
  this->compFuseStdRP = std_rad;
}

void kalmanCoreSetCompSlaveStdRP(kalmanCoreData_t* this, float std_rad)
{
  if (std_rad < 0.0f) std_rad = 0.0f;
  if (std_rad > 1.0f) std_rad = 1.0f;
  this->compSlaveStdRP = std_rad;
}


// q ← q ⊗ δq(gyro*dt)
static void quatIntegrateGyro(float q[4], const Axis3f* gyro, float dt)
{
  float dtwx = dt * gyro->x;
  float dtwy = dt * gyro->y;
  float dtwz = dt * gyro->z;

  float angle = arm_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + COMP_EPS;
  float ca = arm_cos_f32(angle * 0.5f);
  float sa = arm_sin_f32(angle * 0.5f);
  float dq[4] = { ca, sa*dtwx/angle, sa*dtwy/angle, sa*dtwz/angle };

  float q0 = dq[0]*q[0] - dq[1]*q[1] - dq[2]*q[2] - dq[3]*q[3];
  float q1 = dq[1]*q[0] + dq[0]*q[1] + dq[3]*q[2] - dq[2]*q[3];
  float q2 = dq[2]*q[0] - dq[3]*q[1] + dq[0]*q[2] + dq[1]*q[3];
  float q3 = dq[3]*q[0] + dq[2]*q[1] - dq[1]*q[2] + dq[0]*q[3];

  q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
  quatNormalize(q);
}

// q에서 world z=[0,0,1] 을 body frame으로 돌린 값 (중력 방향 예측)
static void quatToBodyZ(const float q[4], float g_est[3])
{
  float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

  float R02 = 2.0f*qx*qz + 2.0f*qw*qy;
  float R12 = 2.0f*qy*qz - 2.0f*qw*qx;
  float R22 = qw*qw - qx*qx - qy*qy + qz*qz;

  g_est[0] = R02;
  g_est[1] = R12;
  g_est[2] = R22;
}

static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// complementary attitude 업데이트 (Mahony PI 형태: bias 추정 포함)
static void complementaryUpdate(kalmanCoreData_t* this, const Axis3f* acc, const Axis3f* gyro, float dt)
{
  if (dt <= 0.0f) {
    return;
  }

  // -----------------------------
  // 1) acc norm 체크 (기존 게이트)
  // -----------------------------
  const float ax = acc->x;
  const float ay = acc->y;
  const float az = acc->z;

  const float an2 = ax*ax + ay*ay + az*az;

  // NaN/Inf/0 방어
  if (!isfinite(an2) || an2 < COMP_EPS) {
    // acc가 이상하면 그냥 gyro만 적분
    quatIntegrateGyro(this->qComp, gyro, dt);
    return;
  }

  const float an = arm_sqrt(an2);
  if (an < COMP_ACC_MIN_NORM || an > COMP_ACC_MAX_NORM) {
    // 1g 근처가 아니면(외력/접촉 가능성↑) → gyro만 적분
    quatIntegrateGyro(this->qComp, gyro, dt);
    return;
  }

  // -----------------------------
  // 2) measured gravity (body)
  // -----------------------------
  const float inv_an = 1.0f / an;
  float g_meas[3] = { -ax * inv_an, -ay * inv_an, -az * inv_an }; // gravity ≈ -acc

  // predicted gravity from current qComp
  float g_est[3];
  quatToBodyZ(this->qComp, g_est);

  // -----------------------------
  // 3) error = cross(g_est, g_meas)
  //   roll/pitch only (ez = 0)
  // -----------------------------
  float ex = g_est[1]*g_meas[2] - g_est[2]*g_meas[1];
  float ey = g_est[2]*g_meas[0] - g_est[0]*g_meas[2];
  // yaw는 acc로 보정하지 않음

  // -----------------------------
  // 4) I-term (bias estimator)
  //   "준정지"에서만 적분 (gyro gate)
  // -----------------------------
  const float gx = gyro->x;
  const float gy = gyro->y;
  const float gz = gyro->z;
  const float gnorm = arm_sqrt(gx*gx + gy*gy + gz*gz) + COMP_EPS;

  if (gnorm < this->compGyroGate) {
    // bias += Ki * e * dt
    this->compBias[0] += this->compKi * ex * dt;
    this->compBias[1] += this->compKi * ey * dt;
    this->compBias[2]  = 0.0f;

    // saturate (중요!)
    const float lim = this->compBiasLimit;
    this->compBias[0] = clampf(this->compBias[0], -lim, lim);
    this->compBias[1] = clampf(this->compBias[1], -lim, lim);
  }
  // gate를 통과 못하면 bias는 유지(적분 정지)

  // -----------------------------
  // 5) corrected omega
  //   ω_corr = gyro + Kp*e + bias
  // -----------------------------
  Axis3f omegaCorr;
  omegaCorr.x = gx + this->compKp * ex + this->compBias[0];
  omegaCorr.y = gy + this->compKp * ey + this->compBias[1];
  omegaCorr.z = gz; // yaw는 그대로 (bias도 안 씀)

  // -----------------------------
  // 6) integrate corrected omega
  // -----------------------------
  quatIntegrateGyro(this->qComp, &omegaCorr, dt);
}


// API 구현

void kalmanCoreSetSlaveAttitudeToComplementary(kalmanCoreData_t* this, bool enable)
{
  this->slaveKalmanToComplementary = enable;
}

void kalmanCoreSetCompKp(kalmanCoreData_t* this, float kp)
{
  // 안전장치: 음수/너무 큰 값 방지 (원하면 범위 조절)
  if (kp < 0.0f) {
    kp = 0.0f;
  }
  if (kp > 50.0f) {
    kp = 50.0f;
  }
  this->compKp = kp;
}

void kalmanCoreSetCompKi(kalmanCoreData_t* this, float ki)
{
  // 안전장치 (원하면 범위 조절)
  if (ki < 0.0f) {
    ki = 0.0f;
  }
  if (ki > 5.0f) {   // Mahony I는 너무 크게 잡으면 바로 발산/드리프트 유발 가능
    ki = 5.0f;
  }
  this->compKi = ki;
}



void kalmanCoreGetComplementaryQuat(const kalmanCoreData_t* this, float q_out[4])
{
  q_out[0] = this->qComp[0];
  q_out[1] = this->qComp[1];
  q_out[2] = this->qComp[2];
  q_out[3] = this->qComp[3];
}
// ----------------------------------------------------------------------


// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// Small number epsilon, to prevent dividing by zero
#define EPS (1e-6f)

__attribute__((used))
void kalmanCoreDefaultParams(kalmanCoreParams_t* params)
{
  *params = (kalmanCoreParams_t){
    KALMAN_CORE_DEFAULT_PARAMS_INIT
  };
}


void kalmanCoreSetAttitudeOutputMode(kalmanCoreData_t* this, uint8_t mode)
{
  if (mode > 2) mode = 0;   // 안전장치: 0/1/2만 허용
  this->attOutMode = mode;
}

void kalmanCoreInit(kalmanCoreData_t *this, const kalmanCoreParams_t *params, const uint32_t nowMs)
{
  // Reset all data to 0 (like upon system reset)
  memset(this, 0, sizeof(kalmanCoreData_t));
  this->attOutMode = 0; // default: kalman
  this->S[KC_STATE_X] = params->initialX;
  this->S[KC_STATE_Y] = params->initialY;
  this->S[KC_STATE_Z] = params->initialZ;
//  this->S[KC_STATE_PX] = 0;
//  this->S[KC_STATE_PY] = 0;
//  this->S[KC_STATE_PZ] = 0;
//  this->S[KC_STATE_D0] = 0;
//  this->S[KC_STATE_D1] = 0;
//  this->S[KC_STATE_D2] = 0;

  // reset the attitude quaternion
  this->initialQuaternion[0] = arm_cos_f32(params->initialYaw / 2);
  this->initialQuaternion[1] = 0.0;
  this->initialQuaternion[2] = 0.0;
  this->initialQuaternion[3] = arm_sin_f32(params->initialYaw / 2);
  for (int i = 0; i < 4; i++) { this->q[i] = this->initialQuaternion[i]; }

  // SEUK
  // --- complementary attitude도 동일하게 시작 ---
  for (int i = 0; i < 4; i++) { this->qComp[i] = this->initialQuaternion[i]; }
  this->lastCompUpdateMs = nowMs;
  this->slaveKalmanToComplementary = false;
  // ---------------------------------------------
  // ===== Mahony I (hard-coded defaults) =====
  this->compBias[0] = 0.0f;
  this->compBias[1] = 0.0f;
  this->compBias[2] = 0.0f;

  this->compKp = 1.0f;
  this->compKi        = 0.05f;  // ★ 시작점: 0.02~0.2 사이가 보통 안전
  this->compBiasLimit = 0.30f;  // bias saturation [rad/s]
  this->compGyroGate  = 1.50f;  // gyro norm gate [rad/s] (준정지에서만 적분)

  this->fuseComplementaryToKalman = false;
  this->compFuseStdRP = 0.05f;   // 예: 0.05 rad ~ 2.9 deg
  this->compSlaveStdRP = 0.10f;  // 예: 0.10 rad ~ 5.7 deg

  // then set the initial rotation matrix to the identity. This only affects
  // the first prediction step, since in the finalization, after shifting
  // attitude errors into the attitude state, the rotation matrix is updated.
  for(int i=0; i<3; i++) { for(int j=0; j<3; j++) { this->R[i][j] = i==j ? 1 : 0; }}

  for (int i=0; i< KC_STATE_DIM; i++) {
    for (int j=0; j < KC_STATE_DIM; j++) {
      this->P[i][j] = 0; // set covariances to zero (diagonals will be changed from zero in the next section)
    }
  }

  // initialize state variances
  this->P[KC_STATE_X][KC_STATE_X]  = powf(params->stdDevInitialPosition_xy, 2);
  this->P[KC_STATE_Y][KC_STATE_Y]  = powf(params->stdDevInitialPosition_xy, 2);
  this->P[KC_STATE_Z][KC_STATE_Z]  = powf(params->stdDevInitialPosition_z, 2);

  this->P[KC_STATE_PX][KC_STATE_PX] = powf(params->stdDevInitialVelocity, 2);
  this->P[KC_STATE_PY][KC_STATE_PY] = powf(params->stdDevInitialVelocity, 2);
  this->P[KC_STATE_PZ][KC_STATE_PZ] = powf(params->stdDevInitialVelocity, 2);

  this->P[KC_STATE_D0][KC_STATE_D0] = powf(params->stdDevInitialAttitude_rollpitch, 2);
  this->P[KC_STATE_D1][KC_STATE_D1] = powf(params->stdDevInitialAttitude_rollpitch, 2);
  this->P[KC_STATE_D2][KC_STATE_D2] = powf(params->stdDevInitialAttitude_yaw, 2);

  this->Pm.numRows = KC_STATE_DIM;
  this->Pm.numCols = KC_STATE_DIM;
  this->Pm.pData = (float*)this->P;

  this->baroReferenceHeight = 0.0;

  this->isUpdated = false;
  this->lastPredictionMs = nowMs;
  this->lastProcessNoiseUpdateMs = nowMs;
}

void kalmanCoreScalarUpdate(kalmanCoreData_t* this, arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise)
{
  // The Kalman gain as a column vector
  NO_DMA_CCM_SAFE_ZERO_INIT static float K[KC_STATE_DIM];
  static arm_matrix_instance_f32 Km = {KC_STATE_DIM, 1, (float *)K};

  // Temporary matrices for the covariance updates
  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN3d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN3m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN3d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float HTd[KC_STATE_DIM * 1];
  static arm_matrix_instance_f32 HTm = {KC_STATE_DIM, 1, HTd};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float PHTd[KC_STATE_DIM * 1];
  static arm_matrix_instance_f32 PHTm = {KC_STATE_DIM, 1, PHTd};

  ASSERT(Hm->numRows == 1);
  ASSERT(Hm->numCols == KC_STATE_DIM);

  // ====== INNOVATION COVARIANCE ======

  mat_trans(Hm, &HTm);
  mat_mult(&this->Pm, &HTm, &PHTm); // PH'
  float R = stdMeasNoise*stdMeasNoise;
  float HPHR = R; // HPH' + R
  for (int i=0; i<KC_STATE_DIM; i++) { // Add the element of HPH' to the above
    HPHR += Hm->pData[i]*PHTd[i]; // this obviously only works if the update is scalar (as in this function)
  }
  ASSERT(!isnan(HPHR));

  // ====== MEASUREMENT UPDATE ======
  // Calculate the Kalman gain and perform the state update
  for (int i=0; i<KC_STATE_DIM; i++) {
    K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
    this->S[i] = this->S[i] + K[i] * error; // state update
  }
  assertStateNotNaN(this);

  // ====== COVARIANCE UPDATE ======
  mat_mult(&Km, Hm, &tmpNN1m); // KH
  for (int i=0; i<KC_STATE_DIM; i++) { tmpNN1d[KC_STATE_DIM*i+i] -= 1; } // KH - I
  mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
  mat_mult(&tmpNN1m, &this->Pm, &tmpNN3m); // (KH - I)*P
  mat_mult(&tmpNN3m, &tmpNN2m, &this->Pm); // (KH - I)*P*(KH - I)'
  assertStateNotNaN(this);
  // add the measurement variance and ensure boundedness and symmetry
  // TODO: Why would it hit these bounds? Needs to be investigated.
  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float v = K[i] * R * K[j];
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i] + v; // add measurement noise
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }

  assertStateNotNaN(this);

  this->isUpdated = true;
}

void kalmanCoreUpdateWithPKE(kalmanCoreData_t* this, arm_matrix_instance_f32 *Hm, arm_matrix_instance_f32 *Km, arm_matrix_instance_f32 *P_w_m, float error)
{
    // kalman filter update with weighted covariance matrix P_w_m, kalman gain Km, and innovation error
    // Temporary matrices for the covariance updates
    static float tmpNN1d[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, (float *)tmpNN1d};
    for (int i=0; i<KC_STATE_DIM; i++){
        this->S[i] = this->S[i] + Km->pData[i] * error;
    }
    // ====== COVARIANCE UPDATE ====== //
    mat_mult(Km, Hm, &tmpNN1m);                 // KH,  the Kalman Gain and H are the updated Kalman Gain and H
    mat_scale(&tmpNN1m, -1.0f, &tmpNN1m);       //  I-KH
    for (int i=0; i<KC_STATE_DIM; i++) { tmpNN1d[i][i] = 1.0f + tmpNN1d[i][i]; }
    float Ppo[KC_STATE_DIM][KC_STATE_DIM]={0};
    arm_matrix_instance_f32 Ppom = {KC_STATE_DIM, KC_STATE_DIM, (float *)Ppo};
    mat_mult(&tmpNN1m, P_w_m, &Ppom);          // Pm = (I-KH)*P_w_m
    memcpy(this->P, Ppo, sizeof(this->P));

    assertStateNotNaN(this);

    for (int i=0; i<KC_STATE_DIM; i++) {
        for (int j=i; j<KC_STATE_DIM; j++) {
        float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
        if (isnan(p) || p > MAX_COVARIANCE) {
            this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
        } else if ( i==j && p < MIN_COVARIANCE ) {
            this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
        } else {
            this->P[i][j] = this->P[j][i] = p;
            }
        }
    }
    assertStateNotNaN(this);

    this->isUpdated = true;
}

void kalmanCoreUpdateWithBaro(kalmanCoreData_t *this, const kalmanCoreParams_t *params, float baroAsl, bool quadIsFlying)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  h[KC_STATE_Z] = 1;

  if (!quadIsFlying || this->baroReferenceHeight < 1) {
    //TODO: maybe we could track the zero height as a state. Would be especially useful if UWB anchors had barometers.
    this->baroReferenceHeight = baroAsl;
  }

  float meas = (baroAsl - this->baroReferenceHeight);
  kalmanCoreScalarUpdate(this, &H, meas - this->S[KC_STATE_Z], params->measNoiseBaro);
}

static void predictDt(kalmanCoreData_t* this, const kalmanCoreParams_t *params, Axis3f *acc, Axis3f *gyro, float dt, bool quadIsFlying)
{
  /* Here we discretize (euler forward) and linearise the quadrocopter dynamics in order
   * to push the covariance forward.
   *
   * QUADROCOPTER DYNAMICS (see paper):
   *
   * \dot{x} = R(I + [[d]])p
   * \dot{p} = f/m * e3 - [[\omega]]p - g(I - [[d]])R^-1 e3 //drag negligible
   * \dot{d} = \omega
   *
   * where [[.]] is the cross-product matrix of .
   *       \omega are the gyro measurements
   *       e3 is the column vector [0 0 1]'
   *       I is the identity
   *       R is the current attitude as a rotation matrix
   *       f/m is the mass-normalized motor force (acceleration in the body's z direction)
   *       g is gravity
   *       x, p, d are the quad's states
   * note that d (attitude error) is zero at the beginning of each iteration,
   * since error information is incorporated into R after each Kalman update.
   */

  // The linearized update matrix
  NO_DMA_CCM_SAFE_ZERO_INIT static float A[KC_STATE_DIM][KC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 Am = { KC_STATE_DIM, KC_STATE_DIM, (float *)A}; // linearized dynamics for covariance update;

  // Temporary matrices for the covariance updates
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  float dt2 = dt*dt;

  // ====== DYNAMICS LINEARIZATION ======
  // Initialize as the identity
  A[KC_STATE_X][KC_STATE_X] = 1;
  A[KC_STATE_Y][KC_STATE_Y] = 1;
  A[KC_STATE_Z][KC_STATE_Z] = 1;

  A[KC_STATE_PX][KC_STATE_PX] = 1;
  A[KC_STATE_PY][KC_STATE_PY] = 1;
  A[KC_STATE_PZ][KC_STATE_PZ] = 1;

  A[KC_STATE_D0][KC_STATE_D0] = 1;
  A[KC_STATE_D1][KC_STATE_D1] = 1;
  A[KC_STATE_D2][KC_STATE_D2] = 1;

  // position from body-frame velocity
  A[KC_STATE_X][KC_STATE_PX] = this->R[0][0]*dt;
  A[KC_STATE_Y][KC_STATE_PX] = this->R[1][0]*dt;
  A[KC_STATE_Z][KC_STATE_PX] = this->R[2][0]*dt;

  A[KC_STATE_X][KC_STATE_PY] = this->R[0][1]*dt;
  A[KC_STATE_Y][KC_STATE_PY] = this->R[1][1]*dt;
  A[KC_STATE_Z][KC_STATE_PY] = this->R[2][1]*dt;

  A[KC_STATE_X][KC_STATE_PZ] = this->R[0][2]*dt;
  A[KC_STATE_Y][KC_STATE_PZ] = this->R[1][2]*dt;
  A[KC_STATE_Z][KC_STATE_PZ] = this->R[2][2]*dt;

  // position from attitude error
  A[KC_STATE_X][KC_STATE_D0] = (this->S[KC_STATE_PY]*this->R[0][2] - this->S[KC_STATE_PZ]*this->R[0][1])*dt;
  A[KC_STATE_Y][KC_STATE_D0] = (this->S[KC_STATE_PY]*this->R[1][2] - this->S[KC_STATE_PZ]*this->R[1][1])*dt;
  A[KC_STATE_Z][KC_STATE_D0] = (this->S[KC_STATE_PY]*this->R[2][2] - this->S[KC_STATE_PZ]*this->R[2][1])*dt;

  A[KC_STATE_X][KC_STATE_D1] = (- this->S[KC_STATE_PX]*this->R[0][2] + this->S[KC_STATE_PZ]*this->R[0][0])*dt;
  A[KC_STATE_Y][KC_STATE_D1] = (- this->S[KC_STATE_PX]*this->R[1][2] + this->S[KC_STATE_PZ]*this->R[1][0])*dt;
  A[KC_STATE_Z][KC_STATE_D1] = (- this->S[KC_STATE_PX]*this->R[2][2] + this->S[KC_STATE_PZ]*this->R[2][0])*dt;

  A[KC_STATE_X][KC_STATE_D2] = (this->S[KC_STATE_PX]*this->R[0][1] - this->S[KC_STATE_PY]*this->R[0][0])*dt;
  A[KC_STATE_Y][KC_STATE_D2] = (this->S[KC_STATE_PX]*this->R[1][1] - this->S[KC_STATE_PY]*this->R[1][0])*dt;
  A[KC_STATE_Z][KC_STATE_D2] = (this->S[KC_STATE_PX]*this->R[2][1] - this->S[KC_STATE_PY]*this->R[2][0])*dt;

  // body-frame velocity from body-frame velocity
  A[KC_STATE_PX][KC_STATE_PX] = 1; //drag negligible
  A[KC_STATE_PY][KC_STATE_PX] =-gyro->z*dt;
  A[KC_STATE_PZ][KC_STATE_PX] = gyro->y*dt;

  A[KC_STATE_PX][KC_STATE_PY] = gyro->z*dt;
  A[KC_STATE_PY][KC_STATE_PY] = 1; //drag negligible
  A[KC_STATE_PZ][KC_STATE_PY] =-gyro->x*dt;

  A[KC_STATE_PX][KC_STATE_PZ] =-gyro->y*dt;
  A[KC_STATE_PY][KC_STATE_PZ] = gyro->x*dt;
  A[KC_STATE_PZ][KC_STATE_PZ] = 1; //drag negligible

  // body-frame velocity from attitude error
  A[KC_STATE_PX][KC_STATE_D0] =  0;
  A[KC_STATE_PY][KC_STATE_D0] = -GRAVITY_MAGNITUDE*this->R[2][2]*dt;
  A[KC_STATE_PZ][KC_STATE_D0] =  GRAVITY_MAGNITUDE*this->R[2][1]*dt;

  A[KC_STATE_PX][KC_STATE_D1] =  GRAVITY_MAGNITUDE*this->R[2][2]*dt;
  A[KC_STATE_PY][KC_STATE_D1] =  0;
  A[KC_STATE_PZ][KC_STATE_D1] = -GRAVITY_MAGNITUDE*this->R[2][0]*dt;

  A[KC_STATE_PX][KC_STATE_D2] = -GRAVITY_MAGNITUDE*this->R[2][1]*dt;
  A[KC_STATE_PY][KC_STATE_D2] =  GRAVITY_MAGNITUDE*this->R[2][0]*dt;
  A[KC_STATE_PZ][KC_STATE_D2] =  0;

  // attitude error from attitude error
  /**
   * At first glance, it may not be clear where the next values come from, since they do not appear directly in the
   * dynamics. In this prediction step, we skip the step of first updating attitude-error, and then incorporating the
   * new error into the current attitude (which requires a rotation of the attitude-error covariance). Instead, we
   * directly update the body attitude, however still need to rotate the covariance, which is what you see below.
   *
   * This comes from a second order approximation to:
   * Sigma_post = exps(-d) Sigma_pre exps(-d)'
   *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
   * where d is the attitude error expressed as Rodriges parameters, ie. d0 = 1/2*gyro.x*dt under the assumption that
   * d = [0,0,0] at the beginning of each prediction step and that gyro.x is constant over the sampling period
   *
   * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
   * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
   */
  float d0 = gyro->x*dt/2;
  float d1 = gyro->y*dt/2;
  float d2 = gyro->z*dt/2;

  A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
  A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
  A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;

  A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
  A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
  A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;

  A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
  A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
  A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;


  // ====== COVARIANCE UPDATE ======
  mat_mult(&Am, &this->Pm, &tmpNN1m); // A P
  mat_trans(&Am, &tmpNN2m); // A'
  mat_mult(&tmpNN1m, &tmpNN2m, &this->Pm); // A P A'
  // Process noise is added after the return from the prediction step

  // ====== PREDICTION STEP ======
  // The prediction depends on whether we're on the ground, or in flight.
  // When flying, the accelerometer directly measures thrust (hence is useless to estimate body angle while flying)

  float dx, dy, dz;
  float tmpSPX, tmpSPY, tmpSPZ;
  float zacc;

  if (quadIsFlying) // only acceleration in z direction
  {
    // Use accelerometer and not commanded thrust, as this has proper physical units
    zacc = acc->z;

    // position updates in the body frame (will be rotated to inertial frame)
    dx = this->S[KC_STATE_PX] * dt;
    dy = this->S[KC_STATE_PY] * dt;
    dz = this->S[KC_STATE_PZ] * dt + zacc * dt2 / 2.0f; // thrust can only be produced in the body's Z direction

    // position update
    this->S[KC_STATE_X] += this->R[0][0] * dx + this->R[0][1] * dy + this->R[0][2] * dz;
    this->S[KC_STATE_Y] += this->R[1][0] * dx + this->R[1][1] * dy + this->R[1][2] * dz;
    this->S[KC_STATE_Z] += this->R[2][0] * dx + this->R[2][1] * dy + this->R[2][2] * dz - GRAVITY_MAGNITUDE * dt2 / 2.0f;

    // keep previous time step's state for the update
    tmpSPX = this->S[KC_STATE_PX];
    tmpSPY = this->S[KC_STATE_PY];
    tmpSPZ = this->S[KC_STATE_PZ];

    // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
    this->S[KC_STATE_PX] += dt * (gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][0]);
    this->S[KC_STATE_PY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][1]);
    this->S[KC_STATE_PZ] += dt * (zacc + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * this->R[2][2]);
  }
  else // Acceleration can be in any direction, as measured by the accelerometer. This occurs, eg. in freefall or while being carried.
  {
    // position updates in the body frame (will be rotated to inertial frame)
    dx = this->S[KC_STATE_PX] * dt + acc->x * dt2 / 2.0f;
    dy = this->S[KC_STATE_PY] * dt + acc->y * dt2 / 2.0f;
    dz = this->S[KC_STATE_PZ] * dt + acc->z * dt2 / 2.0f; // thrust can only be produced in the body's Z direction

    // position update
    this->S[KC_STATE_X] += this->R[0][0] * dx + this->R[0][1] * dy + this->R[0][2] * dz;
    this->S[KC_STATE_Y] += this->R[1][0] * dx + this->R[1][1] * dy + this->R[1][2] * dz;
    this->S[KC_STATE_Z] += this->R[2][0] * dx + this->R[2][1] * dy + this->R[2][2] * dz - GRAVITY_MAGNITUDE * dt2 / 2.0f;

    // keep previous time step's state for the update
    tmpSPX = this->S[KC_STATE_PX];
    tmpSPY = this->S[KC_STATE_PY];
    tmpSPZ = this->S[KC_STATE_PZ];

    // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
    this->S[KC_STATE_PX] += dt * (acc->x + gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][0]);
    this->S[KC_STATE_PY] += dt * (acc->y - gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][1]);
    this->S[KC_STATE_PZ] += dt * (acc->z + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * this->R[2][2]);
  }

  // attitude update (rotate by gyroscope), we do this in quaternions
  // this is the gyroscope angular velocity integrated over the sample period
  float dtwx = dt*gyro->x;
  float dtwy = dt*gyro->y;
  float dtwz = dt*gyro->z;

  // compute the quaternion values in [w,x,y,z] order
  float angle = arm_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + EPS;
  float ca = arm_cos_f32(angle/2.0f);
  float sa = arm_sin_f32(angle/2.0f);
  float dq[4] = {ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle};

  float tmpq0;
  float tmpq1;
  float tmpq2;
  float tmpq3;

  // rotate the quad's attitude by the delta quaternion vector computed above
  tmpq0 = dq[0]*this->q[0] - dq[1]*this->q[1] - dq[2]*this->q[2] - dq[3]*this->q[3];
  tmpq1 = dq[1]*this->q[0] + dq[0]*this->q[1] + dq[3]*this->q[2] - dq[2]*this->q[3];
  tmpq2 = dq[2]*this->q[0] - dq[3]*this->q[1] + dq[0]*this->q[2] + dq[1]*this->q[3];
  tmpq3 = dq[3]*this->q[0] + dq[2]*this->q[1] - dq[1]*this->q[2] + dq[0]*this->q[3];

  if (! quadIsFlying) {
    float keep = 1.0f - params->attitudeReversion;

    tmpq0 = keep * tmpq0 + params->attitudeReversion * this->initialQuaternion[0];
    tmpq1 = keep * tmpq1 + params->attitudeReversion * this->initialQuaternion[1];
    tmpq2 = keep * tmpq2 + params->attitudeReversion * this->initialQuaternion[2];
    tmpq3 = keep * tmpq3 + params->attitudeReversion * this->initialQuaternion[3];
  }

  // normalize and store the result
  float norm = arm_sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + EPS;
  this->q[0] = tmpq0/norm; this->q[1] = tmpq1/norm; this->q[2] = tmpq2/norm; this->q[3] = tmpq3/norm;
  assertStateNotNaN(this);

  this->isUpdated = true;
}

void kalmanCorePredict(kalmanCoreData_t* this, const kalmanCoreParams_t *params,
                       Axis3f *acc, Axis3f *gyro, const uint32_t nowMs, bool quadIsFlying) {
  float dt = (nowMs - this->lastPredictionMs) / 1000.0f;
  predictDt(this, params, acc, gyro, dt, quadIsFlying);
  this->lastPredictionMs = nowMs;

  // --- complementary attitude도 동일한 nowMs 기준으로 업데이트 ---
  float dtComp = (nowMs - this->lastCompUpdateMs) / 1000.0f;
  complementaryUpdate(this, acc, gyro, dtComp);
  this->lastCompUpdateMs = nowMs;
}


static void addProcessNoiseDt(kalmanCoreData_t *this, const kalmanCoreParams_t *params, float dt)
{
  this->P[KC_STATE_X][KC_STATE_X] += powf(params->procNoiseAcc_xy*dt*dt + params->procNoiseVel*dt + params->procNoisePos, 2);  // add process noise on position
  this->P[KC_STATE_Y][KC_STATE_Y] += powf(params->procNoiseAcc_xy*dt*dt + params->procNoiseVel*dt + params->procNoisePos, 2);  // add process noise on position
  this->P[KC_STATE_Z][KC_STATE_Z] += powf(params->procNoiseAcc_z*dt*dt + params->procNoiseVel*dt + params->procNoisePos, 2);  // add process noise on position

  this->P[KC_STATE_PX][KC_STATE_PX] += powf(params->procNoiseAcc_xy*dt + params->procNoiseVel, 2); // add process noise on velocity
  this->P[KC_STATE_PY][KC_STATE_PY] += powf(params->procNoiseAcc_xy*dt + params->procNoiseVel, 2); // add process noise on velocity
  this->P[KC_STATE_PZ][KC_STATE_PZ] += powf(params->procNoiseAcc_z*dt + params->procNoiseVel, 2); // add process noise on velocity

  this->P[KC_STATE_D0][KC_STATE_D0] += powf(params->measNoiseGyro_rollpitch * dt + params->procNoiseAtt, 2);
  this->P[KC_STATE_D1][KC_STATE_D1] += powf(params->measNoiseGyro_rollpitch * dt + params->procNoiseAtt, 2);
  this->P[KC_STATE_D2][KC_STATE_D2] += powf(params->measNoiseGyro_yaw * dt + params->procNoiseAtt, 2);

  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }

  assertStateNotNaN(this);
}

void kalmanCoreAddProcessNoise(kalmanCoreData_t *this, const kalmanCoreParams_t *params, const uint32_t nowMs) {
  float dt = (nowMs - this->lastProcessNoiseUpdateMs) / 1000.0f;
  if (dt > 0.0f) {
    addProcessNoiseDt(this, params, dt);
    this->lastProcessNoiseUpdateMs = nowMs;
  }
}

bool kalmanCoreFinalize(kalmanCoreData_t* this)
{
  // Only finalize if data is updated
  if (! this->isUpdated) {
    return false;
  }

 // (A) Fuse complementary roll/pitch as a measurement into D0/D1
  if (this->fuseComplementaryToKalman) {
    fuseCompRollPitchToKalman(this, this->compFuseStdRP);
  }

  // Matrix to rotate the attitude covariances once updated
  NO_DMA_CCM_SAFE_ZERO_INIT static float A[KC_STATE_DIM][KC_STATE_DIM];
  static arm_matrix_instance_f32 Am = {KC_STATE_DIM, KC_STATE_DIM, (float *)A};

  // Temporary matrices for the covariance updates
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  // Incorporate the attitude error (Kalman filter state) with the attitude
  float v0 = this->S[KC_STATE_D0];
  float v1 = this->S[KC_STATE_D1];
  float v2 = this->S[KC_STATE_D2];

  // Move attitude error into attitude if any of the angle errors are large enough
  if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) > 0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 && fabsf(v2) < 10))
  {
    float angle = arm_sqrt(v0*v0 + v1*v1 + v2*v2) + EPS;
    float ca = arm_cos_f32(angle / 2.0f);
    float sa = arm_sin_f32(angle / 2.0f);
    float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

    // rotate the quad's attitude by the delta quaternion vector computed above
    float tmpq0 = dq[0] * this->q[0] - dq[1] * this->q[1] - dq[2] * this->q[2] - dq[3] * this->q[3];
    float tmpq1 = dq[1] * this->q[0] + dq[0] * this->q[1] + dq[3] * this->q[2] - dq[2] * this->q[3];
    float tmpq2 = dq[2] * this->q[0] - dq[3] * this->q[1] + dq[0] * this->q[2] + dq[1] * this->q[3];
    float tmpq3 = dq[3] * this->q[0] + dq[2] * this->q[1] - dq[1] * this->q[2] + dq[0] * this->q[3];

    // normalize and store the result
    float norm = arm_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3) + EPS;
    this->q[0] = tmpq0 / norm;
    this->q[1] = tmpq1 / norm;
    this->q[2] = tmpq2 / norm;
    this->q[3] = tmpq3 / norm;

    /** Rotate the covariance, since we've rotated the body
     *
     * This comes from a second order approximation to:
     * Sigma_post = exps(-d) Sigma_pre exps(-d)'
     *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
     * where d is the attitude error expressed as Rodriges parameters, ie. d = tan(|v|/2)*v/|v|
     *
     * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
     * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
     */

    float d0 = v0/2; // the attitude error vector (v0,v1,v2) is small,
    float d1 = v1/2; // so we use a first order approximation to d0 = tan(|v0|/2)*v0/|v0|
    float d2 = v2/2;

    A[KC_STATE_X][KC_STATE_X] = 1;
    A[KC_STATE_Y][KC_STATE_Y] = 1;
    A[KC_STATE_Z][KC_STATE_Z] = 1;

    A[KC_STATE_PX][KC_STATE_PX] = 1;
    A[KC_STATE_PY][KC_STATE_PY] = 1;
    A[KC_STATE_PZ][KC_STATE_PZ] = 1;

    A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
    A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
    A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;

    A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
    A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
    A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;

    A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
    A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
    A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

    mat_trans(&Am, &tmpNN1m); // A'
    mat_mult(&Am, &this->Pm, &tmpNN2m); // AP
    mat_mult(&tmpNN2m, &tmpNN1m, &this->Pm); //APA'
  }

  // SEUK
  // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
  // slaveKalmanToComplementary 가 켜져 있으면 Kalman 내부 attitude도 complementary에 동기화
  const float* qUse = this->slaveKalmanToComplementary ? this->qComp : this->q;

  if (this->slaveKalmanToComplementary) {
    // Kalman 내부 q 도 complementary 로 덮어써서 다음 스텝에서 일관되게 사용
    this->q[0] = this->qComp[0];
    this->q[1] = this->qComp[1];
    this->q[2] = this->qComp[2];
    this->q[3] = this->qComp[3];

  this->S[KC_STATE_D0] = 0.0f;
  this->S[KC_STATE_D1] = 0.0f;

    // -------- (B) Covariance handling after attitude injection --------
    // D0/D1 (roll/pitch attitude-error states) are now inconsistent with injected q.
    // We "uncorrelate" D0/D1 from all other states and reset their variances.
    const float var_rp = fmaxf(this->compSlaveStdRP * this->compSlaveStdRP, MIN_COVARIANCE);

    for (int i = 0; i < KC_STATE_DIM; i++) {
      this->P[KC_STATE_D0][i] = 0.0f;
      this->P[i][KC_STATE_D0] = 0.0f;
      this->P[KC_STATE_D1][i] = 0.0f;
      this->P[i][KC_STATE_D1] = 0.0f;
    }

    this->P[KC_STATE_D0][KC_STATE_D0] = var_rp;
    this->P[KC_STATE_D1][KC_STATE_D1] = var_rp;

    // NOTE: D2(yaw)는 건드리지 않음 (legacy yaw-consistency 유지)
    // ---------------------------------------------------------------
  }

  this->R[0][0] = qUse[0] * qUse[0] + qUse[1] * qUse[1] - qUse[2] * qUse[2] - qUse[3] * qUse[3];
  this->R[0][1] = 2 * qUse[1] * qUse[2] - 2 * qUse[0] * qUse[3];
  this->R[0][2] = 2 * qUse[1] * qUse[3] + 2 * qUse[0] * qUse[2];

  this->R[1][0] = 2 * qUse[1] * qUse[2] + 2 * qUse[0] * qUse[3];
  this->R[1][1] = qUse[0] * qUse[0] - qUse[1] * qUse[1] + qUse[2] * qUse[2] - qUse[3] * qUse[3];
  this->R[1][2] = 2 * qUse[2] * qUse[3] - 2 * qUse[0] * qUse[1];

  this->R[2][0] = 2 * qUse[1] * qUse[3] - 2 * qUse[0] * qUse[2];
  this->R[2][1] = 2 * qUse[2] * qUse[3] + 2 * qUse[0] * qUse[1];
  this->R[2][2] = qUse[0] * qUse[0] - qUse[1] * qUse[1] - qUse[2] * qUse[2] + qUse[3] * qUse[3];


  // reset the attitude error
  this->S[KC_STATE_D0] = 0;
  this->S[KC_STATE_D1] = 0;
  this->S[KC_STATE_D2] = 0;

  // enforce symmetry of the covariance matrix, and ensure the values stay bounded
  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }

  assertStateNotNaN(this);

  this->isUpdated = false;
  return true;
}

// SEUK
void kalmanCoreExternalizeState(const kalmanCoreData_t* this, state_t *state, const Axis3f *acc)
{
  float qAttLocal[4];
  const float* qAtt = this->q; // default

  if (this->attOutMode == 0) {
    qAtt = this->q;        // 0: kalman full
  } else if (this->attOutMode == 1) {
    qAtt = this->qComp;    // 1: comp full
  } else {
    // 2: mixed (tilt from comp, yaw from kalman) - quaternion level
    buildMixedQuatYawKalmanTiltComp(this->q, this->qComp, qAttLocal);
    qAtt = qAttLocal;
  }

  // --- 이하 너 기존 코드 그대로: qAtt로 R 계산/vel/acc/attitude/quaternion export ---
  float qw = qAtt[0], qx = qAtt[1], qy = qAtt[2], qz = qAtt[3];

  float R00 = qw*qw + qx*qx - qy*qy - qz*qz;
  float R01 = 2 * qx*qy - 2 * qw*qz;
  float R02 = 2 * qx*qz + 2 * qw*qy;

  float R10 = 2 * qx*qy + 2 * qw*qz;
  float R11 = qw*qw - qx*qx + qy*qy - qz*qz;
  float R12 = 2 * qy*qz - 2 * qw*qx;

  float R20 = 2 * qx*qz - 2 * qw*qy;
  float R21 = 2 * qy*qz + 2 * qw*qx;
  float R22 = qw*qw - qx*qx - qy*qy + qz*qz;

  state->position = (point_t){ .x = this->S[KC_STATE_X], .y = this->S[KC_STATE_Y], .z = this->S[KC_STATE_Z] };

  state->velocity = (velocity_t){
      .x = R00*this->S[KC_STATE_PX] + R01*this->S[KC_STATE_PY] + R02*this->S[KC_STATE_PZ],
      .y = R10*this->S[KC_STATE_PX] + R11*this->S[KC_STATE_PY] + R12*this->S[KC_STATE_PZ],
      .z = R20*this->S[KC_STATE_PX] + R21*this->S[KC_STATE_PY] + R22*this->S[KC_STATE_PZ]
  };

  state->acc = (acc_t){
      .x = R00*acc->x + R01*acc->y + R02*acc->z,
      .y = R10*acc->x + R11*acc->y + R12*acc->z,
      .z = R20*acc->x + R21*acc->y + R22*acc->z - 1
  };

  float yaw   = atan2f(2*(qx*qy+qw*qz) , qw*qw + qx*qx - qy*qy - qz*qz);
  float pitch = asinf(-2*(qx*qz - qw*qy));
  float roll  = atan2f(2*(qy*qz+qw*qx) , qw*qw - qx*qx - qy*qy + qz*qz);

  state->attitude = (attitude_t){
      .roll  = roll*RAD_TO_DEG,
      .pitch = -pitch*RAD_TO_DEG,
      .yaw   = yaw*RAD_TO_DEG
  };

  state->attitudeQuaternion = (quaternion_t){
      .w = qAtt[0], .x = qAtt[1], .y = qAtt[2], .z = qAtt[3]
  };

  assertStateNotNaN(this);
}


// Reset a state to 0 with max covariance
// If called often, this decouples the state to the rest of the filter
static void decoupleState(kalmanCoreData_t* this, kalmanCoreStateIdx_t state)
{
  // Set all covariance to 0
  for(int i=0; i<KC_STATE_DIM; i++) {
    this->P[state][i] = 0;
    this->P[i][state] = 0;
  }
  // Set state variance to maximum
  this->P[state][state] = MAX_COVARIANCE;
  // set state to zero
  this->S[state] = 0;
}

void kalmanCoreDecoupleXY(kalmanCoreData_t* this)
{
  decoupleState(this, KC_STATE_X);
  decoupleState(this, KC_STATE_PX);
  decoupleState(this, KC_STATE_Y);
  decoupleState(this, KC_STATE_PY);
}
