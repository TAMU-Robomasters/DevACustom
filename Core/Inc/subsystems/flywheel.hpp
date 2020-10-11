#pragma once

#include "cmsis_os.h"
#include "information/device.hpp"
#include "information/pid.hpp"
#include "stm32f4xx_hal.h"

namespace flywheel {

class flywheelMotor : public pwmMotor {
private:
    pidInstance* PID;

public:
    flywheelMotor(TIM_HandleTypeDef* tim, int tim_channel, GPIO_TypeDef* GPIOx, int GPIO_Pin, int lR, int uR, float32_t lC, float32_t uC) : pwmMotor(tim, tim_channel, GPIOx, GPIO_Pin, lR, uR) {}
    flywheelMotor(TIM_HandleTypeDef* tim, int tim_channel, GPIO_TypeDef* GPIOx, int GPIO_Pin, int lR, int uR) : pwmMotor(tim, tim_channel, GPIOx, GPIO_Pin, lR, uR) {}
    flywheelMotor(TIM_HandleTypeDef* tim, int tim_channel, GPIO_TypeDef* GPIOx, int GPIO_Pin) : pwmMotor(tim, tim_channel, GPIOx, GPIO_Pin, 533, 1083) {}
};

enum flywheelStates {
    notRunning,
    running,
};
extern flywheelStates currState;

extern void task();

extern void update();

extern void act();

extern double calcSlewDRpm(double currFw1Speed, double currFw2Speed, double targetFw1Speed, double targetFw2Speed);

} // namespace flywheel
