#pragma once

#include "imu/bsp_imu.h"

extern imu_t imu;

namespace userIMU {

extern void imuInit();

extern void imuUpdate();

extern float imuRoll();
extern float imuPitch();
extern float imuYaw();

} // namespace userIMU
