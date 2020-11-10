#pragma once

#include "cmsis_os.h"
#include "information/device.hpp"
#include "information/pid.hpp"
#include <math.h>

#define rotor_angle_max 8191;
#define radsPerTick (2 * PI) / rotor_angle_max;

namespace gimbal {

class gimbalMotor : public canMotor {
private:
    pidInstance* PID;
    double currAngle; //RADIANS

public:
    gimbalMotor(int16_t ID, float32_t lC, float32_t uC) : canMotor(ID, -100, 100) {}
    gimbalMotor(int16_t ID, float32_t lC, float32_t uC, pidInstance& pid) : canMotor(ID, -100, 100), PID(&pid) {}
    gimbalMotor(int16_t ID, pidInstance& pid) : canMotor(ID, -100, 100), PID(&pid) {}

    void setPID(pidInstance& pid) {
        PID = &pid;
    }

    double getCurrAngle() {
        return currAngle;
    }

    void setCurrAngle(double ticks) {
        currAngle = ticks * radsPerTick;
    }
};

extern gimbalMotor yawMotor, pitchMotor;

extern pidInstance yawPosPid, pitchPosPid;

enum gimbalStates {
    notRunning,
    running
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

} // namespace gimbal

extern double radToDeg(double angle);
extern double degToRad(double angle);
