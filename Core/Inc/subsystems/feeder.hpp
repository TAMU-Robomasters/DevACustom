#pragma once

#include "cmsis_os.h"
//#include "information/can_protocol.hpp"

namespace feeder {

extern float feederPower;

enum feederStates {
    notRunning = 'x',
    running = 'r'
};
extern feederStates currState;

extern uint16_t angle;
extern int16_t speed;
extern int16_t torque_current;
extern int8_t temp;

extern void task();

extern void update();

extern void act();

extern void indicator();

} // namespace feeder
