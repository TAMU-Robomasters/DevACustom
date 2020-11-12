#include "subsystems/chassis.hpp"
#include "arm_math.h"
#include "information/can_protocol.hpp"
#include "information/pid.hpp"
#include "information/pwm_protocol.hpp"
#include "init.hpp"

namespace chassis {

/*
Task Assignment: Given an angle from front of the robot (0-360) and a Power(0, 100), calculate the power for each chassis motor to move in that current direction 
Due: Wednesday the 23rd by 11:59Pm
*/

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
    int t = 0;
    bool looped = false;
    if (t < 1000 && looped == false) {      //Arbitrary number of 2 seconds for loops   
        currState = patrol;
        t++;
    }
    else {
        currState = notRunning;
        looped = true;
        t--;
    }
    if (t == 0) {
        looped = false;
    }
    // Update encoders
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
        double power = velPid.loop(static_cast<double>(c1Motor.getFeedback()->rotor_speed) / 19.0);
        c1Motor.setPower(power);
        c2Motor.setPower(power);
        c3Motor.setPower(power);
        c4Motor.setPower(power);
        // if current control, power will be set in the CAN task
        // this will change when we have things to put here
        break;
    case patrol:
        bool patrolLoop = false;
        if (true) {     //FIXME: ENCODERS       if (encoders == ticksToEndOfRail) within a tolerance
            patrolLoop = true;
        }
        else if (false) {  //FIXME: ENCODERS       if (encoders == 0) within a tolerance
            patrolLoop = false;
        }
        if (patrolLoop == false) { 
            double power = velPid.loop(static_cast<double>(c1Motor.getFeedback()->rotor_speed) / 19.0);
            c1Motor.setPower(power);
            c2Motor.setPower(power);
            c3Motor.setPower(power);
            c4Motor.setPower(power);
        }
        else if (patrolLoop == true) {
            double power = -velPid.loop(static_cast<double>(c1Motor.getFeedback()->rotor_speed) / 19.0);
            c1Motor.setPower(power);
            c2Motor.setPower(power);
            c3Motor.setPower(power);
            c4Motor.setPower(power);
        }
        break;
    }
}

void rcToPower(double angle, double magnitude) {
    // Computes the appropriate fraction of the wheel's motor power

    // Sine and cosine of math.h take angle in radians as input value
    c1Motor.setPower(magnitude * sqrt(2.0) * 0.5 * (cos(angle) + sin(angle)));
    c2Motor.setPower(magnitude * sqrt(2.0) * 0.5 * (sin(angle) - cos(angle)));
    c3Motor.setPower(magnitude * sqrt(2.0) * 0.5 * (sin(angle) - cos(angle)));
    c4Motor.setPower(magnitude * sqrt(2.0) * 0.5 * (cos(angle) + sin(angle)));
}

} // namespace chassis
