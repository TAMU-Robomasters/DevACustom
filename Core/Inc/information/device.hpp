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

template <typename T>
float radToDeg(T r) {
    const float pi = PI;
    return (float)((r / pi) * 180);
}

template <typename T>
float degToRad(T d) {
    const float pi = PI;
    return (float)((d / 180) * pi);
}

class Motor {
private:
    float32_t power; // input power, usually from -100 to 100
        // This is instantiated to 0 in Motor, if 0 is not safety for the particular motor then change that in the subclass
    float32_t lowerClamp; // input clamp
    float32_t upperClamp; // input clamp

public:
    Motor(float32_t lC, float32_t uC) : power(0), lowerClamp(lC), upperClamp(uC) {}

    float32_t clamp(double p) {
        if (p < lowerClamp) {
            return lowerClamp;
        }
        if (p > upperClamp) {
            return upperClamp;
        }
        return static_cast<float32_t>(p);
    }

    float32_t getPower() {
        return power;
    }

    void setPower(double p) {
        power = clamp(p);
    }
};

class canMotor : public Motor {
private:
    int16_t canID;
    userCAN::motorFeedback_t canFeedback;
    float radsPerTick;
    float motorGearing;

public:
    canMotor(int16_t ID, float32_t lC, float32_t uC, float angleTicksMax = (2 * PI), float motorGearing = 1) : Motor(lC, uC), canID(ID), radsPerTick((2 * PI) / angleTicksMax), motorGearing(motorGearing) {}

    int16_t getID() {
        return canID;
    }

    userCAN::motorFeedback_t* getFeedback() {
        return &canFeedback;
    }

    float getSpeed() {
        return static_cast<float>(canFeedback.rotor_speed) / motorGearing; // in rpm
    }

    float getAngle() {
        return static_cast<float>(canFeedback.rotor_angle * radsPerTick); // in radians
    }

    float getCurrent() {
        return static_cast<float>(canFeedback.torque_current);
    }

    float getTemp() {
        return static_cast<float>(canFeedback.temp); // in C
    }
};

class pwmMotor {
private:
    float32_t duty;
    TIM_HandleTypeDef* tim_handle;
    int tim_channel;
    GPIO_TypeDef* GPIOx;
    int GPIO_Pin;
    int lowerRange;
    int upperRange;

public:
    pwmMotor(TIM_HandleTypeDef* tim, int tim_c, GPIO_TypeDef* gpiox, int gpio_pin, int lR, int uR)
        : tim_handle(tim), tim_channel(tim_c), GPIOx(gpiox), GPIO_Pin(gpio_pin), lowerRange(lR), upperRange(uR) {
        setPower(0);
    }

    float32_t clamp(double p) {
        if (p < lowerRange) {
            return lowerRange;
        }
        if (p > upperRange) {
            return upperRange;
        }
        return static_cast<float32_t>(p);
    }

    float32_t getPower() {
        return duty;
    }

    void setPower(double p) {
        //p range is from -100 to 100
        // effective max and min are 1083, 533
        int dutyRange = upperRange - lowerRange;

        int unclamped = int(lowerRange + (0.01 * p * dutyRange));

        duty = clamp(unclamped);

        switch (tim_channel) {
        case 1:
            tim_handle->Instance->CCR1 = duty;
            break;
        case 2:
            tim_handle->Instance->CCR2 = duty;
            break;
        case 3:
            tim_handle->Instance->CCR3 = duty;
            break;
        case 4:
            tim_handle->Instance->CCR4 = duty;
            break;
        }
    }

    void initESC() {
        // htim2.Instance->CCR1 = 1000;
        setPower(0);
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
        osDelay(1000);
        setPower(0);
        // htim2.Instance->CCR1 = 500;
        osDelay(1000);
        setPower(0);
    }
};
