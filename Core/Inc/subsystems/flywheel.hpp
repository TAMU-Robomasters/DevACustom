#pragma once

#include "cmsis_os.h"

namespace flywheel {

extern float f1Power;
extern float f2Power;

enum flywheelStates {
    notRunning = 'x',
    running = 'r'
};
extern flywheelStates currState;

extern void task();

extern void update();

extern void act();

} // namespace flywheel
