#include "imu/imu_protocol.hpp"
#include "imu/bsp_imu.h"

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
    return imu.rol;
}

float imuPitch() {
    return imu.pit;
}

float imuYaw() {
    return imu.yaw;
}

} // namespace userIMU
