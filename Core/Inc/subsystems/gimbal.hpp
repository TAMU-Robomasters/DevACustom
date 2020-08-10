#pragma once

#include "cmsis_os.h"

namespace gimbal {

extern float yawPower;
extern float pitchPower;

enum gimbalStates {
    notRunning = 'x',
    running = 'r'
};
extern gimbalStates currState;

extern void task();

extern void update();

extern void act();

} // namespace gimbal
