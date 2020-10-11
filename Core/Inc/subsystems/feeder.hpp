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

enum feedingStatus {
    ready,
    feeding,
    waiting
};
extern feedingStatus currFeedingStatus;
extern uint32_t statusExpire;

extern uint32_t feedingPeriod;
extern uint32_t waitingPeriod;

extern float feederOnPower;

extern uint16_t angle;
extern int16_t speed;
extern int16_t torque_current;
extern int8_t temp;

extern void task();

extern void update();

extern void act();

extern void indicator();

} // namespace feeder
