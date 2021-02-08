#pragma once

#include "stm32f4xx_hal.h"

enum class pidType { position,
                     velocity,
                     tEmPeRaTuRe };

class pidInstance {

private:
    pidType type;
    float kP, kI, kD;
    float lastLoopTime;
    float lastError;
    float errorSum;
    float target;
    float currInput;
    float lastInput;
    float output;
    float dtTimeout;

public:
    pidInstance(float p = 0, float i = 0, float d = 0) : kP(p), kI(i), kD(d), lastLoopTime(0), dtTimeout(250) {}
    pidInstance(pidType pT = pidType::velocity, float p = 0, float i = 0, float d = 0) : type(pT), kP(p), kI(i), kD(d), lastLoopTime(0) {}

    void loop() {

        float currTime = HAL_GetTick();
        float error = target - currInput;
        float dT = (lastLoopTime == 0) ? 0 : (currTime - lastLoopTime);
        if (dT > dtTimeout) {
            dT = 0;
        }

        float derivative = (dT == 0) ? 0 : (-1 * (lastInput - currInput)) / (dT);
        //instead of using lastError - error, we use -(lastInput - currInput) to minimize output spikes when the target changes
        errorSum += error * dT;

        float output = (kP * error) + (kI * errorSum) + (kD * derivative);

        lastLoopTime = currTime;
        lastError = error;
        lastInput = currInput;

        this->output = output;
    }

    float getTarget() {
        return this->target;
    }

    void setTarget(double t) {
        this->target = t;
    }

    float getCurrInput() {
        return this->currInput;
    }

    void setCurrInput(float in) {
        this->currInput = in;
    }

    float getOutput() {
        return this->output;
    }
};
