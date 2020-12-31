#include "information/uart_protocol.hpp"

#include "cmsis_os.h"

#include "init.hpp"

#include "subsystems/chassis.hpp"
#include "subsystems/feeder.hpp"
#include "subsystems/gimbal.hpp"

namespace userUART {

void numOut(UART_HandleTypeDef* huart, uint8_t num) {
    uint8_t* numptr = &num;
    HAL_UART_Transmit(huart, numptr, sizeof(num), 25);
}

void stringOut(UART_HandleTypeDef* huart, char buffer[]) {
    HAL_UART_Transmit(huart, (uint8_t*)buffer, sizeof(buffer), 25);
}

void newlnOut(UART_HandleTypeDef* huart) {
    char newline[2] = "\n";
    HAL_UART_Transmit(huart, (uint8_t*)newline, 2, 25);
}

void motorFeedbackOut(UART_HandleTypeDef* huart, userCAN::motorFeedback_t* data) {

    uint16_t angle = data->rotor_angle;
    uint16_t speed = data->rotor_speed + 32768;
    uint16_t current = data->torque_current + 32768;
    // adds 32768 to shift int16_t values to uint16_t, shifted back when data processed
    uint8_t temp = data->temp;

    uint8_t buffer[9];
    buffer[0] = 'M';          // start byte to tell our processing program that a data "package" has started
    buffer[1] = (angle >> 8); // bitshifts uint16_t to split into "high" 8 bit int
    buffer[2] = (angle);      // makes "low" 8 bit int
    buffer[3] = (speed >> 8);
    buffer[4] = (speed);
    buffer[5] = (current >> 8);
    buffer[6] = (current);
    buffer[7] = (temp);
    buffer[8] = 'Z'; // end byte just for assurance, isn't technically useful right now

    HAL_UART_Transmit(huart, buffer, 9, 50);
}

void yawInfoOut(UART_HandleTypeDef* huart, userCAN::motorFeedback_t* data) {

    //uint16_t angle = data->rotor_angle;
    //uint16_t speed = data->rotor_speed + 32768;
    // uint16_t current = data->torque_current + 32768;
    uint16_t sentPower = static_cast<int16_t>(gimbal::yawMotor.getPower()) + 32768;
    uint16_t currAngle = static_cast<int16_t>(radToDeg(gimbal::yawMotor.getCurrAngle())) + 32768;
    uint16_t PIDError = static_cast<int16_t>(radToDeg(gimbal::yawPosPid.getTarget() - gimbal::yawPosPid.getCurrInput())) + 32768;
    // adds 32768 to shift int16_t values to uint16_t, shifted back when data processed
    uint8_t temp = data->temp;

    uint8_t buffer[9];
    buffer[0] = 'M';              // start byte to tell our processing program that a data "package" has started
    buffer[1] = (currAngle >> 8); // bitshifts uint16_t to split into "high" 8 bit int
    buffer[2] = (currAngle);      // makes "low" 8 bit int
    buffer[3] = (sentPower >> 8);
    buffer[4] = (sentPower);
    buffer[5] = (PIDError >> 8);
    buffer[6] = (PIDError);
    buffer[7] = (temp);
    buffer[8] = 'Z'; // end byte just for assurance, isn't technically useful right now

    HAL_UART_Transmit(huart, buffer, 9, 50);
}

void task() {

    for (;;) {
        receive();

        send();
        // for sending messages over UART

        osDelay(20);
    }
}

void receive() {
}
 
void send() {
    userUART::yawInfoOut(&huart6, gimbal::yawMotor.getFeedback());
}

} // namespace userUART
