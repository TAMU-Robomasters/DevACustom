#include "subsystems/flywheel.hpp"
#include "init.hpp"
#include "information/rc_protocol.h"
#include <arm_math.h>

namespace flywheel {

flywheelStates currState = notRunning;

double angularAccLimit = 0.1; // defined as rpm/10ms^2

flywheelMotor flywheel1(&htim2, 1, POWER1_CTRL_GPIO_Port, POWER1_CTRL_Pin);
flywheelMotor flywheel2(&htim2, 2, POWER3_CTRL_GPIO_Port, POWER3_CTRL_Pin);
flywheelMotor flywheel3(&htim2, 3, POWER1_CTRL_GPIO_Port, POWER1_CTRL_Pin);
flywheelMotor flywheel4(&htim2, 4, POWER3_CTRL_GPIO_Port, POWER3_CTRL_Pin);

void task() {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    osDelay(500);
    flywheel1.initESC();
    flywheel2.initESC();
    //flywheel3.initESC();
    //flywheel4.initESC();

    for (;;) {
        update();

        act();
        // set power to global variable here, message is actually sent with the others in the CAN task

        osDelay(10);
    }
}

void update() {
		currState = running;
		if (getSwitch(switchType::right) == switchPosition::up){
				currState = notRunning;
				if (getSwitch(switchType::left) == switchPosition::up || getSwitch(switchType::left) == switchPosition::mid) {
						currState = running;
				}
		}
}

void act() {
    switch (currState) {
    case notRunning:
        flywheel1.setPower(0);
        flywheel2.setPower(0);
        flywheel3.setPower(0);
        flywheel4.setPower(0);
        break;

    case running:
        float target = 26.15;
        if(flywheel1.getPower() < target){
            flywheel1.setPower(flywheel1.getPower() + 0.01f);
            flywheel2.setPower(flywheel2.getPower() + 0.01f);
            //flywheel3.setPower(flywheel3.getPower() + 0.05f);
            //flywheel4.setPower(flywheel4.getPower() + 0.05f);
        }
        else{
            flywheel1.setPower(target);
            flywheel2.setPower(target);
            //flywheel3.setPower(target);
            //flywheel4.setPower(target);
        }
        //calcSlewDRpm(flywheel1.getPower(), flywheel2.getPower(), 10, 40);
        // obviously this will change when we have actual intelligent things to put here
        break;
    }
}

double calcSlewDRpm(double currFw1Speed, double currFw2Speed, double targetFw1Speed, double targetFw2Speed) {
    double fw1DRPM;
    double fw2DRPM;
    double velLim = angularAccLimit * 10;

    if(angularAccLimit > abs(targetFw1Speed - currFw1Speed) / 10) {
        fw1DRPM = targetFw1Speed - currFw1Speed;
    }
    else if(angularAccLimit < abs(targetFw1Speed - currFw1Speed) / 10) {
        if(targetFw1Speed - currFw1Speed < 0) {
            fw1DRPM = -velLim;
        }
        else if(targetFw1Speed - currFw1Speed > 0) {
            fw1DRPM = velLim;
        }
    }
    else {
        fw1DRPM = 0;
    }

    if(angularAccLimit > abs(targetFw2Speed - currFw2Speed) / 10) {
        fw2DRPM = targetFw2Speed - currFw2Speed;
    }
    else if(angularAccLimit < abs(targetFw2Speed - currFw2Speed) / 10) {
        if(targetFw2Speed - currFw2Speed < 0) {
            fw2DRPM = -velLim;
        }
        else if(targetFw2Speed - currFw2Speed > 0) {
            fw2DRPM = velLim;
        }
    }
    else {
        fw2DRPM = 0;
    }

    flywheel1.setPower(fw1DRPM);
    flywheel2.setPower(fw2DRPM);

    return 0;
}
} // namespace flywheel
