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
};

extern gimbalMotor yawMotor, pitchMotor;

extern pidInstance yawPosPid, pitchPosPid;

enum gimbalStates {
    notRunning,
    aimFromCV,
    idle,
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

extern double normalizePitchAngle();

extern double calculateAngleError(double currAngle, double targetAngle);

extern void sendGimbMessage();
extern void sendGimbMessage(float y);

} // namespace gimbal
