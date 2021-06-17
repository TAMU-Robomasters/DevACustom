#pragma once

#include "imu/bsp_imu.h"

namespace userIMU {

extern void imuInit();

extern void imuUpdate();

extern float imuRoll();
extern float imuPitch();
extern float imuYaw();

extern float imuRollSpeed();
extern float imuPitchSpeed();
extern float imuYawSpeed();

} // namespace userIMU
