#include "subsystems/chassis.hpp"
#include "information/can_protocol.hpp"
#include "information/pid.hpp"
#include "information/pwm_protocol.hpp"
#include "information/rc_protocol.h"
#include "init.hpp"
#include <arm_math.h>

float ch0 = static_cast<float>(rcDataStruct.rc.ch[0]);
float ch1 = static_cast<float>(rcDataStruct.rc.ch[1]);
float ch2 = static_cast<float>(rcDataStruct.rc.ch[2]);
float ch3 = static_cast<float>(rcDataStruct.rc.ch[3]);

namespace chassis {

chassisStates currState = notRunning;
CtrlTypes ctrlType = CURRENT;
// i don't really like this but do i care enough to change it?

pidInstance velPid(pidType::velocity, 0.7, 0.0, 0.0);

chassisMotor c1Motor(userCAN::M3508_M1_ID, velPid);
chassisMotor c2Motor(userCAN::M3508_M2_ID, velPid);
chassisMotor c3Motor(userCAN::M3508_M3_ID, velPid);
chassisMotor c4Motor(userCAN::M3508_M4_ID, velPid);

void task() {

    for (;;) {
        update();

        act();
        // set power to global variable here, message is actually sent with the others in the CAN task

        osDelay(2);
    }
}

void update() {
    if (true) {
        currState = notRunning;
        // will change later based on RC input and sensor based decision making
    }

    float rcSomething = static_cast<float>(rcDataStruct.rc.ch[0]);

    // rcSomething = rcSomething / 1320 * 100;
    // divide by total range and scale to -100, 100

    // velPid.setTarget(rcSomething);
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
        double power = velPid.loop(static_cast<double>(c1Motor.getFeedback()->rotor_speed) / 19.0);
        c1Motor.setPower(power);
        c2Motor.setPower(power);
        c3Motor.setPower(power);
        c4Motor.setPower(power);
        // if current control, power will be set in the CAN task
        // this will change when we have things to put here
        break;
    }
}

void rcToPower(double angle, double magnitude) {
    // Computes the appropriate fraction of the wheel's motor power

    // Sine and cosine of math.h take angle in radians as input value
    c1Motor.setPower(magnitude * sqrt(2.0) * 0.5 * (cos(angle) + sin(angle))); // 
    c2Motor.setPower(magnitude * sqrt(2.0) * 0.5 * (sin(angle) - cos(angle)));
    c3Motor.setPower(magnitude * sqrt(2.0) * 0.5 * (sin(angle) - cos(angle)));
    c4Motor.setPower(magnitude * sqrt(2.0) * 0.5 * (cos(angle) + sin(angle)));
}

} // namespace chassis
