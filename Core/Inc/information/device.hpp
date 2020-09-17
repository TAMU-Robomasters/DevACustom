/* Includes ------------------------------------------------------------------*/
#pragma once

#include "cmsis_os.h"
#include "information/can_protocol.hpp"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
/* 
    Referee System Abstractions?
    Taken from QDU (used in CAN protocol)
*/

#define DEVICE_OK (0)
#define DEVICE_ERR (-1)
#define DEVICE_ERR_NULL (-2)
#define DEVICE_ERR_INITED (-3)
#define DEVICE_ERR_NO_DEV (-4)

#define CAN_GM6020_MAX_ABS_VOLT 30000
#define M3508_MAX_CURRENT 16384
#define M2006_MAX_CURRENT 10000

#define PI 3.14159265358979323846

class Motor {
private:
    float32_t power = 0;  // input power, usually from -100 to 100
    float32_t lowerClamp; // input clamp
    float32_t upperClamp; // input clamp

public:
    Motor(float32_t lC, float32_t uC) : lowerClamp(lC), upperClamp(uC) {}

    float32_t clamp(double p) {
        if (p < lowerClamp) {
            return lowerClamp;
        }
        if (p > upperClamp) {
            return upperClamp;
        }
        return static_cast<float32_t>(p);
    }

    virtual float32_t getPower() {
        return power;
    }

    virtual void setPower(double p) {
        power = clamp(p);
    }
};

class canMotor : public Motor {
private:
    int16_t canID;
    userCAN::motorFeedback_t canFeedback;

public:
    canMotor(int16_t ID, float32_t lC, float32_t uC) : Motor(lC, uC), canID(ID) {}

    int16_t getID() {
        return canID;
    }

    userCAN::motorFeedback_t* getFeedback() {
        return &canFeedback;
    }
};

// class pwmMotor : public Motor {
// private:
//     int upperRange = 1083;
//     int lowerRange = 532;
//     TIM_HandleTypeDef* tim;
//     int tim_channel;
//     GPIO_TypeDef* GPIOx;
//     int GPIO_Pin;

// public:
//     pwmMotor(TIM_HandleTypeDef* tim, int tim_channel, GPIO_TypeDef* GPIOx, int GPIO_Pin, int lR, int uR, float32_t lC, float32_t uC) : Motor(-100, 100) {}

//     void setPower(double p) {
//         // effective max and min are 1083, 533
//         int dutyRange = upperRange - lowerRange;
//         int zeroPoint = (upperRange - lowerRange) / 2;

//         //p /= abs(lowerRange - upperRange) / 2;
//         int duty = int(zeroPoint + p * dutyRange);

//         //p is from -100 to 100
//         //need to scale from lowerRange to upperRange
//         //

//         switch (tim_channel) {
//         case 1:
//             tim->Instance->CCR1 = duty;
//             break;
//         case 2:
//             tim->Instance->CCR2 = duty;
//             break;
//         case 3:
//             tim->Instance->CCR3 = duty;
//             break;
//         case 4:
//             tim->Instance->CCR4 = duty;
//             break;
//         }
//     }

//     void initESC() {
//         // htim2.Instance->CCR1 = 1000;
//         setPower(tim, tim_channel, 1);
//         HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
//         osDelay(1000);
//         setPower(tim, 1, -.058);
//         // htim2.Instance->CCR1 = 500;
//         osDelay(1000);
//     }
// };
