#include "subsystems/chassis.hpp"
#include "information/can_protocol.hpp"
#include "information/pid.hpp"
#include "information/pwm_protocol.hpp"
#include "init.hpp"
#include <math.h>
#include <iostream.h>


namespace chassis {

/*
Task Assignment: Given an angle from front of the robot (0-360) and a Power(0, 100), calculate the power for each chassis motor to move in that current direction 
Due: Wednesday the 23rd by 11:59Pm
*/
chassisStates currState = notRunning;
CtrlTypes ctrlType = CURRENT;
// i don't really like this but do i care enough to change it?

pidInstance velPid(pidInstance::type::velocity, 0.7, 0.0, 0.0);

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
    if (true) {
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
        double power = velPid.loop(static_cast<double>(c1Motor.getFeedback()->rotor_speed) / 19.0);
        c1Motor.setPower(power);
        c2Motor.setPower(power);
        c3Motor.setPower(power);
        c4Motor.setPower(power);
        if (ctrlType == VOLTAGE) {
            userPWM::setPower(&htim2, 1, c1Motor.getPower());
        }
        // if current control, power will be set in the CAN task
        // this will change when we have things to put here
        break;
    }
}

void rcToPower(double angle, double magnitude) {
    // IMPLEMENT THIS
    /* 
    Given an input angle value (in radians) and a magnitude (from 0-100), calculate the motor powers required
    for a 4 wheel mecanum drive. Then, set the power for the motors using the setPower() function in the motor class.
    Assume the wheel configuration looks like this: https://photos.app.goo.gl/jYnKYL16uT51C9XWA
    (disregard all the letters and stuff just look at the direction the mecanum rollers are oriented)
    c1Motor = top left
    c2Motor = top right
    c3Motor = bottom left
    c4Motor = bottom right
    EX: If the input angle is 0/360 and the input magnitude is 50, the corresponding powers should be:
        c1Motor -> 50
        c2Motor -> 50
        c3Motor -> 50
        c4Motor -> 50
        If the input angle is 45 degrees and the input magnitude is 100, the corresponding powers should be:
        c1Motor -> 100
        c2Motor -> 0
        c3Motor -> 0
        c4Motor -> 100
    (the example was in degrees, please use radians)
    */

    if (magnitude > 100 || magnitude < 100){
    	std::cout << "Magnitude is not within specified bounds" << std::endl;
    }

    // Please uncomment this else if block if it is necessary to restrict the angle to 0 and 360.
    /* 
    else if(angle < 0 || angle > 360){
    	std::cout << "Angle is not within specified bounds" << std::endl;
    }
    */

    // Computes the appropriate fraction of the wheel's motor power  
    else{
    	// Sine and cosine of math.h take angle in radians as input value
    	c1Motor.setPower(magnitude*sqrt(2)*0.5*(cos(angle) + sin(angle))); 
	    c2Motor.setPower(magnitude*sqrt(2)*0.5*(sin(angle) - cos(angle)));
	    c3Motor.setPower(magnitude*sqrt(2)*0.5*(sin(angle) - cos(angle)));
	    c4Motor.setPower(magnitude*sqrt(2)*0.5*(cos(angle) + sin(angle)));
	}






} // namespace chassis