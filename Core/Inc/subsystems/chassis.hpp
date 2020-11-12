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
    chassisMotor(int16_t ID, float32_t lC, float32_t uC) : canMotor(ID, -100, 100) {}
    chassisMotor(int16_t ID, float32_t lC, float32_t uC, pidInstance& pid) : canMotor(ID, -100, 100), PID(&pid) {}
    chassisMotor(int16_t ID, pidInstance& pid) : canMotor(ID, -100, 100), PID(&pid) {}

    void setPID(pidInstance& pid) {
        PID = &pid;
    }
};

extern chassisMotor c1Motor, c2Motor, c3Motor, c4Motor;

enum chassisStates {
    notRunning,
    followGimbal,
    manual, 
    patrol
};

enum CtrlTypes {
    CURRENT,
    VOLTAGE,
    SPEED,
};
extern CtrlTypes ctrlType;

extern void pwmInitialize();

extern void task();

extern void update();

extern void act();

extern void rcToPower(double angle, double magnitude);

} // namespace chassis
