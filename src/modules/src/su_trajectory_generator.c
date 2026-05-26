#include "su_trajectory_generator.h"

#include <math.h>

#include "su_params.h"
#include "su_position_trigger.h"

#define SU_TRAJECTORY_RATE_HZ 50
#define SU_PI_F 3.14159265358979323846f

typedef enum {
  SU_TRAJECTORY_SHAPE_NONE = 0,
  SU_TRAJECTORY_SHAPE_CIRCLE = 1,
  SU_TRAJECTORY_SHAPE_SQUARE = 2,
} su_trajectory_shape_t;

typedef struct {
  uint8_t shape;
  float sizeX;
  float sizeY;
  float periodS;
} suTrajectoryConfig_t;

static bool trajectoryActive = false;
static point_t trajectoryOrigin;
static float trajectoryYawDeg = 0.0f;
static float trajectoryElapsedS = 0.0f;

static suTrajectoryConfig_t getTrajectoryConfig(uint8_t trajectoryMode)
{
  suTrajectoryConfig_t config = {
    .shape = SU_TRAJECTORY_SHAPE_NONE,
    .sizeX = 0.0f,
    .sizeY = 0.0f,
    .periodS = 1.0f,
  };

  if (trajectoryMode != SU_TRAJECTORY_NONE) {
    config.shape = su_traj1_shape;
    config.sizeX = su_traj1_size_x;
    config.sizeY = su_traj1_size_y;
    config.periodS = su_traj1_period_s;
  }

  if (config.periodS < 1e-3f) {
    config.periodS = 1e-3f;
  }

  return config;
}

static void updateCircle(const suTrajectoryConfig_t *config, point_t *position, float *yawDeg)
{
  const float omega = 2.0f * SU_PI_F / config->periodS;
  const float phase = omega * trajectoryElapsedS;

  position->x = trajectoryOrigin.x;
  position->y = trajectoryOrigin.y + config->sizeX * cosf(phase);
  position->z = trajectoryOrigin.z + config->sizeY * sinf(phase);
  *yawDeg = trajectoryYawDeg;
}

static void updateSquare(const suTrajectoryConfig_t *config, point_t *position, float *yawDeg)
{
  const float segmentDuration = config->periodS / 4.0f;
  const float wrapped = fmodf(trajectoryElapsedS, config->periodS);
  const int segment = ((int)(wrapped / segmentDuration)) % 4;
  const float alpha = (wrapped - (float)segment * segmentDuration) / segmentDuration;
  const float halfY = config->sizeX * 0.5f;
  const float halfZ = config->sizeY * 0.5f;

  switch (segment) {
    case 0:
      position->y = trajectoryOrigin.y - halfY + config->sizeX * alpha;
      position->z = trajectoryOrigin.z - halfZ;
      break;
    case 1:
      position->y = trajectoryOrigin.y + halfY;
      position->z = trajectoryOrigin.z - halfZ + config->sizeY * alpha;
      break;
    case 2:
      position->y = trajectoryOrigin.y + halfY - config->sizeX * alpha;
      position->z = trajectoryOrigin.z + halfZ;
      break;
    case 3:
    default:
      position->y = trajectoryOrigin.y - halfY;
      position->z = trajectoryOrigin.z + halfZ - config->sizeY * alpha;
      break;
  }

  position->x = trajectoryOrigin.x;
  *yawDeg = trajectoryYawDeg;
}

void suTrajectoryGeneratorInit(void)
{
  trajectoryActive = false;
  trajectoryOrigin.x = 0.0f;
  trajectoryOrigin.y = 0.0f;
  trajectoryOrigin.z = 0.0f;
  trajectoryYawDeg = 0.0f;
  trajectoryElapsedS = 0.0f;
}

void suTrajectoryGeneratorDeactivate(void)
{
  trajectoryActive = false;
}

void suTrajectoryGeneratorStart(uint8_t trajectoryMode, const point_t *origin, float yawDeg)
{
  const suTrajectoryConfig_t config = getTrajectoryConfig(trajectoryMode);

  if (!origin || trajectoryMode == SU_TRAJECTORY_NONE || config.shape == SU_TRAJECTORY_SHAPE_NONE) {
    trajectoryActive = false;
    return;
  }

  trajectoryActive = true;
  trajectoryOrigin = *origin;
  trajectoryYawDeg = yawDeg;
  trajectoryElapsedS = 0.0f;
}

bool suTrajectoryGeneratorIsActive(void)
{
  return trajectoryActive;
}

void suTrajectoryGeneratorUpdate(uint8_t trajectoryMode, stabilizerStep_t stabilizerStep, point_t *position, float *yawDeg)
{
  const suTrajectoryConfig_t config = getTrajectoryConfig(trajectoryMode);

  if (!trajectoryActive || !position || !yawDeg || trajectoryMode == SU_TRAJECTORY_NONE) {
    return;
  }

  if (config.shape == SU_TRAJECTORY_SHAPE_NONE) {
    trajectoryActive = false;
    return;
  }

  if (RATE_DO_EXECUTE(SU_TRAJECTORY_RATE_HZ, stabilizerStep)) {
    trajectoryElapsedS += 1.0f / (float)SU_TRAJECTORY_RATE_HZ;
  }

  switch (config.shape) {
    case SU_TRAJECTORY_SHAPE_CIRCLE:
      updateCircle(&config, position, yawDeg);
      break;
    case SU_TRAJECTORY_SHAPE_SQUARE:
      updateSquare(&config, position, yawDeg);
      break;
    case SU_TRAJECTORY_SHAPE_NONE:
    default:
      trajectoryActive = false;
      break;
  }
}
