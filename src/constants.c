#include <stdlib.h>
#include <math.h>

const float LF = 0.792;
const float LR = 0.738;
const float WHEELBASE = LF + LR;

const float DELTA_T = 0.013486;

const float CAM_UNCERTAINTY = 0.05;
const float CAM_MIN_PERCEPTION_DISTANCE = 1.0;
const float CAM_MAX_PERCEPTION_DISTANCE = 8.0;
const float CAM_HFOV = (M_PI / 180.0) * 110.0;

const float NEW_LANDMARK_THRESHOLD = 0.7;

const float SPEED_UNCERTAINTY = 0.03;
const float STEER_UNCERTAINTY = M_PI / 180.0;
const float ODOMETRY_COV[2][2] = {{SPEED_UNCERTAINTY * SPEED_UNCERTAINTY, 0.0}, {0.0, STEER_UNCERTAINTY * STEER_UNCERTAINTY}};