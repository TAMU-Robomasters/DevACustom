/* Includes ------------------------------------------------------------------*/
#pragma once

#include "information/can_protocol.hpp"

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

class Motor {
private:
    float32_t power = 0;
    float32_t lowerClamp;
    float32_t upperClamp;

public:
    Motor(float32_t lC, float32_t uC) : lowerClamp(lC), upperClamp(uC) {}

    float32_t clamp(double p) {
        if (p > upperClamp) {
            return upperClamp;
        }
        if (p < lowerClamp) {
            return lowerClamp;
        }
        return (float32_t)p;
    }

    float32_t getPower() {
        return power;
    }

    void setPower(double p) {
        power = clamp(p);
    }
};
