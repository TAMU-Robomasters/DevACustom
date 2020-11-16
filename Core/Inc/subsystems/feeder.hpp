#pragma once
// header guard

#include "cmsis_os.h"
//#include "information/can_protocol.hpp"
#include "information/device.hpp"
#include "information/pid.hpp"
#include <math.h>


#define rotor_angle_max 8091;
#define radsPerTick (2 * PI) / rotor_angle_max;

class feederMotor: public canMotor{
	private:
		pidInstance* PID;
		double currAngle; // in radians
	public:
		feederMotor(int16_t ID, float32_t lC, float32_t uC): canMotor(ID, -100, 100){}
		feederMotor(int16_t ID, float32_t lC, float32_t uC, pidInstance& pid) : canMotor(ID, -100, 100), PID(&pid) {}
		feederMotor(int16_t ID, pidInstance& pid) : canMotor(ID, -100, 100), PID(&pid) {}

		void setPID(pidInstance& pid) {
		        PID = &pid;
		}

		double getCurrAngle() {
			return currAngle;
		}

        void setCurrAngle(double ticks) {
        	currAngle = ticks * radsPerTick;
		}

};



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
