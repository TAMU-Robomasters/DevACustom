#pragma once

#include "information/filters.hpp"
#include "stm32f4xx_hal.h"
#include <math.h>

// const double sysTicksPerSecond = 1000;
// const double serviceUpperTimeout = 0.2; // Maximum amount of time before the PID resets
// // NOTE: Clock does not run while paused so time doesn't either
// const double serviceLowerSkip = 0.00001; // Minimum amount of time between PID updates to prevent precision issues

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
    float iWindupBound;
    float derivative;

public:
    pidInstance(float p = 0, float i = 0, float d = 0) : kP(p), kI(i), kD(d), lastLoopTime(0), iWindupBound(100) {}
    pidInstance(pidType pT = pidType::velocity, float p = 0, float i = 0, float d = 0) : type(pT), kP(p), kI(i), kD(d), lastLoopTime(0), iWindupBound(100) {}

    // pidInstance(pidType _type, double p = 0, double i = 0, double d = 0, double _outSafe = 0.0, double _outMin = -100.0, double _outMax = 100.0) :
    //     type(_type), kP(p), kI(i), kD(d), outSafe(_outSafe), outMin(_outMin), outMax(_outMax)
    // 	{
    // 		reset();
    // 	}

    float loop(float currInput) {
        this->currInput = currInput;

        float currTime = HAL_GetTick();
        float error = target - currInput;
        float dT = (lastLoopTime == 0) ? 0 : (currTime - lastLoopTime);

        derivative = (dT == 0) ? 0 : ((currInput - lastInput) / (dT));
        //instead of using lastError - error, we use -(lastInput - currInput) to minimize output spikes when the target changes
        errorSum += error * dT * kI;
        if (std::abs(errorSum) > iWindupBound){
            errorSum = std::copysign(iWindupBound, errorSum);
        }

        float output = (kP * error) + (errorSum) + (kD * derivative);

        lastLoopTime = currTime;
        lastError = error;
        lastInput = currInput;

        this->output = output;
        return output;
    }

    double clamp(double a, double b, double c) {
        if (b < a) {
            return a;
        }
        if (c < b) {
            return c;
        }
        return b;
    }

    // float loop(float current) {
    //     double currTime = HAL_GetTick() / sysTicksPerSecond;
    //     double dT = currTime - lastTime;

    //     // Initial step should be dropped to prevent weird jumps
    //     if (lastTime < 0) {
    //         lastTime = currTime;
    //         lastCurrent = current;
    //         return outSafe;
    //     }

    //     // Short, zero, or negative steps should be skipped
    //     if (dT < serviceLowerSkip) {
    //         return lastOutput;
    //     }

    //     // Long steps should be treated as a reset
    //     if (dT > serviceUpperTimeout) {
    //         reset();
    //         return outSafe;
    //     }

    //     // Calculate error
    //     double error; // Direction & magnitude to grow in
    //                   // if (type == PIDType::angle) {
    //                   //     error = calculateAngleError(current, target);
    //                   // }
    //                   // else {
    //     error = target - current;
    //     // }

    //     // Calculate output P
    //     double outputP = kP * error;

    //     // Calculate output I
    //     integralSum += kI * error * dT; // Reduce chance of integral going crazy during live tuning by multiplying kI each iteration

    //     integralSum = clamp(outMin, integralSum, outMax); // Reduce reset windup
    //     double outputI = integralSum;

    //     // Calculate output D
    //     double derivative = (current - lastCurrent) / (dT); // Uses change in value and not change w.r.t. target to prevent jumps
    //                                                         // if (type == PIDType::angle) {
    //                                                         //     derivative = calculateAngleError(lastCurrent, current);  // This of course assumes it won't be going more than 180 degrees per step, if it is then angle control shouldn't be selected anyways
    //                                                         // }
    //                                                         // else {
    //     derivative = current - lastCurrent;
    //     // }
    //     derivative /= dT;

    //     double outputD = -kD * derivative; // Note that this is negative since we want to dampen not increase, letting all constants be positive

    //     // Sanity check: if the derivative is trying to accelerate us in the direction that we are going, don't apply it
    //     if ((outputD > 0 && derivative > 0) || (outputD < 0 && derivative < 0)) {
    //         outputD = 0;
    //     }

    //     // Apply outputP, outputI, and outputD

    //     double output = outputP + outputI + outputD;

    //     // Possible to-do: limit outputD so it can't turn around motion

    //     // We promised that the output would be bounded
    //     output = clamp(outMin, output, outMax);

    //     lastTime = currTime;
    //     lastCurrent = current;
    //     lastOutput = output;

    //     return output;
    // }

    // void reset() {
    //     lastTime = -1;        // signal reset
    //     lastOutput = outSafe; // turn off motors
    // }

    float getTarget() {
        return this->target;
    }

    void setTarget(double t) {
        this->target = t;
    }

    float getCurrInput() {
        return this->currInput;
    }

    // void setCurrInput(float in) {
    //     this->currInput = in;
    // }

    float getDerivative() {
        return this->derivative;
    }

    float getOutput() {
        return this->output;
    }
};
