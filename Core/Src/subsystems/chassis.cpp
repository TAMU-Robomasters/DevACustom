#include "subsystems/chassis.hpp"
#include "information/can_protocol.hpp"
#include "information/pid.hpp"
#include "information/pwm_protocol.hpp"
#include "init.hpp"

namespace chassis {

chassisStates currState = notRunning;
CtrlTypes ctrlType = CURRENT;
// i don't really like this but do i care enough to change it?

pidInstance velPid(1.0, 0.0, 0.0);

chassisMotor c1Motor(userCAN::M3508_M1_ID, velPid);
chassisMotor c2Motor(userCAN::M3508_M2_ID, velPid);
chassisMotor c3Motor(userCAN::M3508_M3_ID, velPid);
chassisMotor c4Motor(userCAN::M3508_M4_ID, velPid);

void task() {

    if (ctrlType == VOLTAGE) {
        pwmInitialize();
    }

    for (;;) {
        update();

        act();
        // set power to global variable here, message is actually sent with the others in the CAN task

        osDelay(2);
    }
}

void pwmInitialize() {

    userPWM::initChannels();

    userPWM::initESC(&htim2, 1, POWER1_CTRL_GPIO_Port, POWER1_CTRL_Pin);
    // if we're using voltage control, we need to initialize the PWM channels and ESC
}

void update() {
    if (1) {
        currState = manual;
        // will change later based on RC input and sensor based decision making
    }
    velPid.setTarget(200);
    // if button pressed on controller, change state to "followgimbal" or something
}

void act() {
    switch (currState) {
    case notRunning:
        c1Motor.setPower(0);
        c2Motor.setPower(0);
        c3Motor.setPower(0);
        c4Motor.setPower(0);
        break;

    case followGimbal:
        c1Motor.setPower(0);
        c2Motor.setPower(0);
        c3Motor.setPower(0);
        c4Motor.setPower(0);
        // obviously this will change when we have things to put here
        break;

    case manual:
        //double power = velPid.loop(static_cast<double>(c1Motor.getFeedback()->rotor_speed) / 19.0);
        c1Motor.setPower(0);
        c2Motor.setPower(0);
        c3Motor.setPower(0);
        c4Motor.setPower(0);
        if (ctrlType == VOLTAGE) {
            userPWM::setPower(&htim2, 1, c1Motor.getPower());
        }
        // if current control, power will be set in the CAN task
        // this will change when we have things to put here
        break;
    }
}

} // namespace chassis
