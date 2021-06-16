#pragma once

#include "cmsis_os.h"
#include "information/device.hpp"
#include "information/pid.hpp"
#include <math.h>
//#include "information/can_protocol.hpp"

namespace feeder {

class feederMotor : public canMotor {
private:
    pidInstance* PID;

public:
    feederMotor(int16_t ID, float32_t lC, float32_t uC, filter::Kalman filter) : canMotor(ID, -100, 100, filter, 8191, 36) {}
    feederMotor(int16_t ID, float32_t lC, float32_t uC, pidInstance& pid, filter::Kalman filter) : canMotor(ID, -100, 100, filter, 8191, 36), PID(&pid) {}
    feederMotor(int16_t ID, pidInstance& pid, filter::Kalman filter) : canMotor(ID, -100, 100, filter, 8191, 36), PID(&pid) {}

    void setPID(pidInstance& pid) {
        PID = &pid;
    }
};

extern feederMotor agitatorLeft;
extern feederMotor agitatorRight;
extern feederMotor indexer;

enum feederStates {
    notRunning,
		unJam,
    running
};
extern feederStates currState;

extern void task();

extern void update();

extern void act();

} // namespace feeder
