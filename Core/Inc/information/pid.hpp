#pragma once

#include "stm32f4xx_hal.h"

class pidInstance {

public:
    enum type {
        position,
        velocity,
        tEmPeRaTuRe
    };

    pidInstance(double p = 0, double i = 0, double d = 0) : kP(p), kI(i), kD(d), lastLoopTime(0) {}
    pidInstance(type pT = velocity, double p = 0, double i = 0, double d = 0) : pidType(pT), kP(p), kI(i), kD(d), lastLoopTime(0) {}

    double loop(double current) {
        double currTime = HAL_GetTick();
        double error = target - current;
        double dT = (lastLoopTime == 0) ? 0 : (currTime - lastLoopTime);

        double derivative = (dT == 0) ? 0 : (lastError - error) / (dT);
        integral += error * dT;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        lastLoopTime = currTime;
        lastError = error;

        return output;
    }

    void setTarget(double t) {
        target = t;
    }

private:
    type pidType;
    double kP, kI, kD;
    double lastLoopTime;
    double lastError;
    double integral;
    double target;
};
