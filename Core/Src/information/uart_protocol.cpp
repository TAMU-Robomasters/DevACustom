#include "information/uart_protocol.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"

#include "init.hpp"

#include "subsystems/chassis.hpp"
#include "subsystems/feeder.hpp"
#include "subsystems/gimbal.hpp"

using namespace std;
using namespace userUART;

int goodReceive = 0;
uint8_t uart6InBuffer[16];
uint8_t uart7InBuffer[6];
uint8_t uart8InBuffer[2];
uint8_t transmitString[] = "teehee";
uint8_t* word6 = NULL;
int rxCallback6 = 0;
int rxCallback7 = 0;
int rxCallback8 = 0;
int rxAnyCallback = 0;

char stringX[] = "00000";
char stringY[] = "00000";
float angleX = 85*PI/180;
float angleY = 260*PI/180;

int startingIndex = 0;

bool dontUpdateBuffer = false;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    rxAnyCallback++;
    // if (readBuf[0] == 'r') {
    // 	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    // 	HAL_UART_Transmit(&huart6, txString, sizeof(txString), timeout);
    // }
    // else if (readBuf[0] == 's') {
    // 	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    // 	HAL_UART_Transmit(&huart6, txString1, sizeof(txString1), timeout);
    // }
    if (huart == &huart6) {
        rxCallback6++;
				if (!dontUpdateBuffer){
					HAL_UART_Receive_IT(huart, (uint8_t*)uart6InBuffer, 16);
				}
        // for (int i = 0; i < 16; i++) {
        //     waitingForWord = true;
        //     if (userUART::serialBuffer6.enqueue(uart6InBuffer[i])) { // means delimiter recieved
        //         if (word6 != NULL) {
        //             delete[] word6;
        //         }
        //         word6 = userUART::serialBuffer6.getLastWord();
        //        waitingForWord = false;
        //         while (!waitingForWord) {
        //             osDelay(1);
        //         }
        //     }
        // }
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

circularBuffer<serial_buffer_size, uint8_t> serialBuffer6(' ');

bool devBoardHandshake = false;

// void numOut(UART_HandleTypeDef* huart, uint8_t num) {
//     uint8_t* numptr = &num;
//     HAL_UART_Transmit(huart, numptr, sizeof(num), 25);
// }

// void stringOut(UART_HandleTypeDef* huart, char buffer[]) {
//     HAL_UART_Transmit(huart, (uint8_t*)buffer, sizeof(buffer), 25);
// }

// void newlnOut(UART_HandleTypeDef* huart) {
//     char newline[2] = "\n";
//     HAL_UART_Transmit(huart, (uint8_t*)newline, 2, 25);
// }

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

    HAL_UART_Receive_IT(&huart6, (uint8_t*)uart6InBuffer, 1);
    HAL_UART_Receive_IT(&huart7, (uint8_t*)uart7InBuffer, 1);
    HAL_UART_Receive_IT(&huart8, (uint8_t*)uart8InBuffer, 1);
	
    char messagePrefix = 's';
    int tempStringSize = 0;

    for (;;) {
        receive();
        // if (uart6InBuffer[0] == 'r') {
        //     goodReceive = 420;
        // }
        // if (uart6InBuffer[1] == 't') {
        //     goodReceive = 69;
        // }

        // if (!waitingForWord) { // if not waiting for word
        //     goodReceive = 10;
        //     //     if (word6[0] == messagePrefix) { // check if word recieved is the defined message prefix
        //     //         waitingForWord = true;
        //     //         while (waitingForWord) { // wait for next word
        //     //             osDelay(1);
        //     //         }
        //     //         tempStringSize = serialBuffer6.getLastWordSize() - 1;
        //     //         for (int i = 0; i < tempStringSize; i++) { // append chars from sent word to string numX
        //     //             // stringX.append(1, word6[i]);
        //     //         }
        //     //         // angleX = std::atof(stringX.c_str());
        //     //         waitingForWord = true;
        //     //         while (waitingForWord) { // wait for next word
        //     //             osDelay(1);
        //     //         }
        //     //         for (int i = 0; i < serialBuffer6.getLastWordSize() - 1; i++) { // append chars from sent word to string numY
        //     //             // stringY.append(1, word6[i]);
        //     //         }
        //     //         // angleX = std::atof(stringY.c_str());
        //     waitingForWord = true;
        //     //     }
        // }
				
				dontUpdateBuffer = true;
				startingIndex = 0;
				
				//uint8_t copyBuffer[16] = uart6InBuffer[startingIndex];
				
				//while (startingIndex <= 15 && uart6InBuffer[startingIndex] != 's'){
				//	startingIndex++;
				//}
				for (int i = 0; i <= 15; i++){
					if(uart6InBuffer[i] == 's'){
						startingIndex = i;
						break;
					}
				}
				
				//if (uart6InBuffer[0] == 's' && uart6InBuffer[14] == 'e'){
				//	goodReceive = 10;
				//	for (int i = 0; i < 5; i++){
				//		stringX[i] = uart6InBuffer[i + 2];
				//		stringY[i] = uart6InBuffer[i + 8];
				//	}
				//}
				goodReceive = (startingIndex + 2)%16;
				for (int i = 0; i < 5; i++){
						stringX[i] = uart6InBuffer[(startingIndex + 2 + i)%16];
						stringY[i] = uart6InBuffer[(startingIndex + 8 + i)%16];
				}
				//angleX = atof(stringX)/1000;
				//angleY = atof(stringY)/1000;
				//for (int i = 5; i >= 1; i--){
				//int pw = pow(10, i - 1);
				if (uart6InBuffer[startingIndex] == 's'){
					angleX = 10000 * (stringX[0] - '0');
					angleX += 1000 * (stringX[1] - '0');
					angleX += 100 * (stringX[2] - '0');
					angleX += 10 * (stringX[3] - '0');
					angleX += 1 * (stringX[4] - '0');
					angleY = 10000 * (stringY[0] - '0');
					angleY += 1000 * (stringY[1] - '0');
					angleY += 100 * (stringY[2] - '0');
					angleY += 10 * (stringY[3] - '0');
					angleY += 1 * (stringY[4] - '0');
				
					angleX /= 1000;
					angleY /= 1000;
				}
				//else{
				//	angleX = 85*PI/180;
				//	angleY = 260*PI/180;
				//}
				//}
				
				dontUpdateBuffer = false;

        send();
        //HAL_UART_Transmit(&huart7, transmitString, sizeof(transmitString), 100);
        //HAL_UART_Transmit(&huart6, transmitString, sizeof(transmitString), 100);
        // HAL_UART_Transmit(&huart6, transmitString, sizeof(transmitString), 100);
        // for sending messages over UART

        osDelay(5);
    }
}

void receive() {}
 
void send() {
    // HAL_UART_Transmit(&huart7, (uint8_t*)"handshake", sizeof("handshake"), 100);
    // userUART::yawInfoOut(&huart6, gimbal::yawMotor.getFeedback());
}

} // namespace userUART
