#include "imu/imu_protocol.hpp"
#include "imu/bsp_imu.h"
#include "information/device.hpp"

#define RADS_TO_RPM 9.549297f

namespace userIMU {

void imuInit() {
    mpu_device_init();
    init_quaternion();
}

void imuUpdate() {
    mpu_get_data();
    imu_ahrs_update();
    imu_attitude_update();
}

float imuRoll() {
    return degToRad(imu.pit);
}

float imuPitch() {
    return degToRad(imu.rol);
}

float imuYaw() {
    return degToRad(imu.yaw);
}

float imuRollSpeed() {
    return imu.wy * RADS_TO_RPM;
}

float imuPitchSpeed() {
    return imu.wx * RADS_TO_RPM;
}

float imuYawSpeed() {
    return imu.wz * RADS_TO_RPM;
}

} // namespace userIMU
