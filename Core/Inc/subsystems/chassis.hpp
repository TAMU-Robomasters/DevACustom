#pragma once

#include "cmsis_os.h"

namespace chassis {

extern float c1Power;
extern float c2Power;
extern float c3Power;
extern float c4Power;

enum chassisStates {
    notRunning = 'x',
    followGimbal = 'g',
    manual = 'm'
};
extern chassisStates currState;

extern void task();

extern void update();

extern void act();

} // namespace chassis
