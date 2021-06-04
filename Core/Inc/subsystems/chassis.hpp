#pragma once

#include "cmsis_os.h"
#include "information/device.hpp"
#include "information/pid.hpp"
#include "init.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"

namespace chassis {

class chassisMotor : public canMotor {
private:
    pidInstance* PID;

public:
    chassisMotor(int16_t ID, float32_t lC, float32_t uC, filter::Kalman filter) : canMotor(ID, -100, 100, filter, 8191, 19) {}
    chassisMotor(int16_t ID, float32_t lC, float32_t uC, pidInstance& pid, filter::Kalman filter) : canMotor(ID, -100, 100, filter, 8191, 19), PID(&pid) {}
    chassisMotor(int16_t ID, pidInstance& pid, filter::Kalman filter) : canMotor(ID, -100, 100, filter, 8191, 19), PID(&pid) {}

    void setPID(pidInstance& pid) {
        PID = &pid;
    }
};
extern chassisMotor c1Motor;

enum chassisStates {
    notRunning = 0,
    followGimbal = 1,
    manual = 2,
};
extern chassisStates currState;

struct chassisTargets {
    float m1;
};

enum CtrlTypes {
    CURRENT,
    VOLTAGE,
    SPEED,
};
extern CtrlTypes ctrlType;

extern filter::Kalman chassisVelFilter;

extern void pwmInitialize();

extern void task();

extern void update();

extern void act();

extern void rcToPower(double angle, double magnitude, double yaw);

extern void sendChassisMessage(float m1);

} // namespace chassis
