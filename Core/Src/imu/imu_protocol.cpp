#include "imu/imu_protocol.hpp"
#include "imu/bsp_imu.h"
#include "information/device.hpp"

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

} // namespace userIMU
