#pragma once

#include "cmsis_os.h"
#include "information/device.hpp"
#include "information/pid.hpp"
#include "init.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"

namespace chassis {

class chassisMotor : public Motor {
private:
    int16_t canID;
    userCAN::motorFeedback_t canFeedback;
    pidInstance* PID;

public:
    chassisMotor(int16_t ID, float32_t lC, float32_t uC) : Motor(-M3508_MAX_CURRENT, M3508_MAX_CURRENT), canID(ID) {}
    chassisMotor(int16_t ID, float32_t lC, float32_t uC, pidInstance& pid) : Motor(-M3508_MAX_CURRENT, M3508_MAX_CURRENT), canID(ID), PID(&pid) {}
    chassisMotor(int16_t ID, pidInstance& pid) : Motor(-M3508_MAX_CURRENT, M3508_MAX_CURRENT), canID(ID), PID(&pid) {}

    int16_t getID() {
        return canID;
    }

    userCAN::motorFeedback_t* getFeedback() {
        return &canFeedback;
    }

    void setPID(pidInstance& pid) {
        PID = &pid;
    }
};

extern chassisMotor c1Motor, c2Motor, c3Motor, c4Motor;

enum chassisStates {
    notRunning,
    followGimbal,
    manual
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

} // namespace chassis
