#include "information/uart_protocol.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"

#include "init.hpp"

#include "circularBuffer.cpp"
#include "subsystems/chassis.hpp"
#include "subsystems/feeder.hpp"
#include "subsystems/gimbal.hpp"

#define LONG_TIME 0xffff

using namespace std;
using namespace userUART;

int goodReceive = 0;
volatile uint8_t uart6InBuffer[1];
volatile uint8_t uart7InBuffer[6];
volatile uint8_t uart8InBuffer[2];
uint8_t transmitString[] = "teehee";
volatile uint8_t* word6 = nullptr;
volatile int rxCallback6 = 0;
volatile int rxCallback7 = 0;
volatile int rxCallback8 = 0;
volatile int rxAnyCallback = 0;

volatile char stringX[] = "00000";
volatile char stringY[] = "00000";
volatile uint8_t aimArray[5];
volatile float angleX = 0;            //0*PI/180; used to be the initial angle for yaw (0)
volatile float angleY = 0 * PI / 180; //used to be initial angle for pitch (115)

SemaphoreHandle_t uart6Semaphore = NULL;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    rxAnyCallback++;

    if (huart == &huart6) {
        //if (uart6Semaphore != NULL && xSemaphoreTakeFromISR(uart6Semaphore, NULL) == pdTRUE) {
        HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
        rxCallback6++;
        HAL_UART_Receive_IT(huart, (uint8_t*)uart6InBuffer, 1);
        if (userUART::serialBuffer6.enqueue(uart6InBuffer[0])) {
            word6 = userUART::serialBuffer6.getLastWord();
            // goodReceive = userUART::serialBuffer6.getLastWordSize();

            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(uart6Semaphore, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken == pdTRUE) {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
    if (huart == &huart7) {
        rxCallback7++;
        HAL_UART_Receive_IT(huart, (uint8_t*)uart7InBuffer, 6);
    }
    if (huart == &huart8) {
        rxCallback8++;
        HAL_UART_Receive_IT(huart, (uint8_t*)uart8InBuffer, 1);
    }
}

namespace userUART {

circularBuffer<serial_buffer_size, uint8_t> serialBuffer6('e');

bool devBoardHandshake = false;

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
    uint16_t currAngle = static_cast<int16_t>(radToDeg(gimbal::yawMotor.getAngle())) + 32768;
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

    uart6Semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(uart6Semaphore);

    HAL_UART_Receive_IT(&huart6, (uint8_t*)uart6InBuffer, 1);
    HAL_UART_Receive_IT(&huart7, (uint8_t*)uart7InBuffer, 1);
    HAL_UART_Receive_IT(&huart8, (uint8_t*)uart8InBuffer, 1);

    for (;;) {
        send();
        for (size_t i = 0; i < serialBuffer6.getLastWordSize(); i++) {
            aimArray[i] = word6[i];
        }
        if (xSemaphoreTake(uart6Semaphore, 0) == pdTRUE) {
            switch (word6[0]) {
            case aimAt:
                goodReceive = 69;
                //angleX = angleY = 0;
                uint16_t x1 = word6[1];
                uint16_t x2 = word6[2];
                uint16_t y1 = word6[3];
                uint16_t y2 = word6[4];
                uint16_t unsnX = (x1 << 8) + x2;
                uint16_t unsnY = (y1 << 8) + y2;
                int16_t snX = unsnX - 32768;
                int16_t snY = unsnY - 32768;
                angleX = static_cast<float>(snX);
                angleY = static_cast<float>(snY);
                //goodReceive = word6[1] - '0';
                // for (unsigned int i = 0; i < 5; i++) {
                //     angleX += word6[i + 1] - '0' /* * pow(10.0f, (int)(5 - i))*/;
                //     stringX[i] = word6[i + 1];
                // }
                break;
            }
        }
        // osDelay(5);
    }
}

void receive() {}
 
void send() {
    // HAL_UART_Transmit(&huart7, (uint8_t*)"handshake", sizeof("handshake"), 100);
    // userUART::yawInfoOut(&huart6, gimbal::yawMotor.getFeedback());
    //HAL_UART_Transmit(&huart6, transmitString, sizeof(transmitString), 100);
    // for sending messages over UART
}

} // namespace userUART
