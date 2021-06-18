#pragma once

#include "cmsis_os.h"
#include "information/device.hpp"
#include "information/pid.hpp"
#include "init.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"

extern float chasMaxRPM;

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
extern chassisMotor c1Motor, c2Motor, c3Motor, c4Motor;

enum chassisStates {
    notRunning = 0,
    followGimbal = 1,
    manual = 2,
    spinToWin = 3,
};
extern chassisStates currState;

struct chassisTargets {
    float m1;
    float m2;
    float m3;
    float m4;
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

extern void rcToPower(double rotate, double forward, double strafe);

extern void sendChassisMessage(float m1, float m2, float m3, float m4);
extern void sendChassisMessage();

extern void chassisPowerUpdate();

} // namespace chassis
