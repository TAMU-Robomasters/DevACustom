#pragma once

#include "cmsis_os.h"
#include "information/device.hpp"
#include "information/pid.hpp"
#include <math.h>

//#define gimbal_angle_max 8191;
// #define radsPerTick (2 * PI) / rotor_angle_max;

namespace gimbal {

class gimbalMotor : public canMotor {
private:
    pidInstance* PID;
    double currAngle; //RADIANS

public:
    gimbalMotor(int16_t ID, float32_t lC, float32_t uC, filter::Kalman filter) : canMotor(ID, -100, 100, filter, 8191, 1) {}
    gimbalMotor(int16_t ID, float32_t lC, float32_t uC, pidInstance& pid, filter::Kalman filter) : canMotor(ID, -100, 100, filter, 8191, 1), PID(&pid) {}
    gimbalMotor(int16_t ID, pidInstance& pid, filter::Kalman filter) : canMotor(ID, -100, 100, filter, 8191, 1), PID(&pid) {}

    void setPID(pidInstance& pid) {
        PID = &pid;
    }

    // double getCurrAngle() {
    //     return currAngle;
    // }

    // void setCurrAngle(double ticks) {
    //     currAngle = ticks * radsPerTick;
    // }
};

extern gimbalMotor yawMotor, pitchMotor;

extern pidInstance yawPosPid, pitchPosPid;

enum gimbalStates {
    notRunning = 0,
    aimFromCV = 1,
    idle = 2,
    imuIdle = 3,
    rcYawLocked = 4,
    rcYawUnlocked = 5,
};
extern gimbalStates currState;

enum CtrlTypes {
    CURRENT,
    VOLTAGE,
    SPEED,
};
extern CtrlTypes ctrlType;

extern void task();

extern void update();

extern void act();

extern double calculateAngleError(double currAngle, double targetAngle);

extern double normalizePitchAngle();

extern void sendGimbMessage();
extern void sendGimbMessage(float y);
extern void sendGimbMessage(float y, float imuData);

} // namespace gimbal
