#include "logic/settledUtils.hpp"
#include "cmsis_os.h"
#include "main.h"
#include <arm_math.h>

bool isSettled(settleType st, float curr, float target, float tolerance) {
    // switch (st) {
    // case position:
        if (fabs(curr - target) < tolerance) {
           return true;
				}
    //    break;

    // case velocity:
    //    break;
    // }
    return false;
}

bool waitUntilSettled(settleType st, float curr, float target, float tolerance) {
    while (!isSettled(st, curr, target, tolerance)) {
        osDelay(2);
    }
    return true;
}
