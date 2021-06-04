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
volatile uint8_t uart7InBuffer[1];
volatile uint8_t uart8InBuffer[1];
uint8_t transmitString[] = "teehee";
volatile uint8_t* jetsonMessage = nullptr;
volatile uint8_t* d2dMessage = nullptr;
volatile int rxCallback6 = 0;
volatile int rxCallback7 = 0;
volatile int rxCallback8 = 0;
volatile int rxAnyCallback = 0;

volatile uint8_t aimArray[5];
volatile float angleX = 0 * PI / 180; //0*PI/180; used to be the initial angle for yaw (0)
volatile float angleY = 0 * PI / 180; //used to be initial angle for pitch (115)

int lagTestStart = 0;
int lagTestEnd = 0;
int lagTestRoundTrip = 0;
uint8_t lagTestArray[] = {'s', 'b', 'e'};

int lastChassisTime, currChassisTime, timeSinceLastChassisMessage;
int lastGimbalTime, currGimbalTime, timeSinceLastGimbalMessage;

SemaphoreHandle_t uart6Semaphore = NULL;
SemaphoreHandle_t uart8Semaphore = NULL;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    rxAnyCallback++;

    if (huart == &huart6) {
        //if (uart6Semaphore != NULL && xSemaphoreTakeFromISR(uart6Semaphore, NULL) == pdTRUE) {
        rxCallback6++;
        HAL_UART_Receive_IT(huart, (uint8_t*)uart6InBuffer, 1);
        if (userUART::jetsonBuffer.enqueue(uart6InBuffer[0])) {
            jetsonMessage = userUART::jetsonBuffer.getLastWord();
            // goodReceive = userUART::jetsonBuffer.getLastWordSize();

            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(uart6Semaphore, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken == pdTRUE) {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
    if (huart == &huart7) {
        rxCallback7++;
        HAL_UART_Receive_IT(huart, (uint8_t*)uart7InBuffer, 1);
    }
    if (huart == &huart8) {
        rxCallback8++;
        HAL_UART_Receive_IT(huart, (uint8_t*)uart8InBuffer, 1);
        if (userUART::d2dBuffer.enqueue(uart8InBuffer[0])) {
            d2dMessage = userUART::d2dBuffer.getLastWord();
            //goodReceive = userUART::d2dBuffer.getLastWordSize();

            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(uart8Semaphore, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken == pdTRUE) {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}

namespace userUART {

QueueHandle_t aimMsgQueue = NULL;
QueueHandle_t gimbMsgQueue = NULL;
QueueHandle_t chassisMsgQueue = NULL;
struct aimMsgStruct txAimMessage;
struct gimbMsgStruct txGimbMessage;
struct chassisMsgStruct txChassisMessage;

circularBuffer<serial_buffer_size, uint8_t> jetsonBuffer('e');
circularBuffer<serial_buffer_size, uint8_t> d2dBuffer('e');

bool devBoardHandshake = false;

void task() {

    aimMsgQueue = xQueueCreate(1, sizeof(&txAimMessage));
    gimbMsgQueue = xQueueCreate(1, sizeof(&txGimbMessage));
    chassisMsgQueue = xQueueCreate(1, sizeof(&txChassisMessage));
    aimMsgStruct* aimMsgPtr;
    gimbMsgStruct* gimbMsgPtr;
    chassisMsgStruct* chassisMsgPtr;

    uart6Semaphore = xSemaphoreCreateBinary();
    uart8Semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(uart6Semaphore);
    xSemaphoreGive(uart8Semaphore);

    HAL_UART_Receive_IT(&huart6, (uint8_t*)uart6InBuffer, 1);
    HAL_UART_Receive_IT(&huart7, (uint8_t*)uart7InBuffer, 1);
    HAL_UART_Receive_IT(&huart8, (uint8_t*)uart8InBuffer, 1);

    lagTestStart = HAL_GetTick();
    //HAL_UART_Transmit(&huart8, (uint8_t*)lagTestArray, sizeof(lagTestArray), 100);

    for (;;) {
        send();
        if (operatingType == primary && xSemaphoreTake(uart6Semaphore, 0) == pdTRUE) {
            switch (jetsonMessage[0]) {
            case aimAt: {
                if (userUART::jetsonBuffer.getLastWordSize() == 5) {
                    uint16_t x1 = jetsonMessage[1];
                    uint16_t x2 = jetsonMessage[2];
                    uint16_t y1 = jetsonMessage[3];
                    uint16_t y2 = jetsonMessage[4];
                    uint16_t unsnX = (x1 << 8) + x2;
                    uint16_t unsnY = (y1 << 8) + y2;
                    int16_t snX = (unsnX - 32768);
                    int16_t snY = (unsnY - 32768);
                    angleX = static_cast<float>(snX) / 10000.0f;
                    angleY = static_cast<float>(snY) / 10000.0f;

                    txAimMessage.prefix = jetsonMessage[0];
                    txAimMessage.disp[0] = angleX;
                    txAimMessage.disp[1] = angleY;
                    aimMsgPtr = &txAimMessage;
                    xQueueSend(aimMsgQueue, (void*)&aimMsgPtr, (TickType_t)0);
                }
                break;
            }
            case structSync: {
                goodReceive = 69;
                lagTestEnd = HAL_GetTick();
                lagTestRoundTrip = lagTestEnd - lagTestStart;
                lagTestStart = HAL_GetTick();
                HAL_UART_Transmit(&huart6, (uint8_t*)lagTestArray, sizeof(lagTestArray), 50);
                break;
            }
            }
        }

        if (xSemaphoreTake(uart8Semaphore, 0) == pdTRUE) {
            switch (d2dMessage[0]) {
            case gimbal: {
                if (userUART::d2dBuffer.getLastWordSize() == 5) {
                    //goodReceive = 69;
                    uint16_t y1 = d2dMessage[2];
                    uint16_t y2 = d2dMessage[3];
                    int16_t snY = (y1 << 8) + y2 - 32768;

                    txGimbMessage.prefix = d2dMessage[0];
                    txGimbMessage.state = static_cast<gimbal::gimbalStates>(d2dMessage[1]);
                    txGimbMessage.yaw = static_cast<float>(snY) / 10000.0f;
                    gimbMsgPtr = &txGimbMessage;
                    xQueueSend(gimbMsgQueue, (void*)&gimbMsgPtr, (TickType_t)0);

                    lastGimbalTime = currGimbalTime;
                    currGimbalTime = HAL_GetTick();
                    timeSinceLastGimbalMessage = currGimbalTime - lastGimbalTime;
                }
                break;
            }
            case chassis: {
                if (userUART::d2dBuffer.getLastWordSize() == 4) {
                    goodReceive = 69;
                    chassis::chassisStates state = static_cast<chassis::chassisStates>(d2dMessage[1]);
                    uint16_t c1_1 = d2dMessage[2];
                    uint16_t c1_2 = d2dMessage[3];

                    int16_t snC1 = (c1_1 << 8) + c1_2 - 32768;

                    txChassisMessage.prefix = d2dMessage[0];
                    txChassisMessage.state = static_cast<chassis::chassisStates>(d2dMessage[1]);
                    txChassisMessage.m1 = static_cast<float>(snC1) / 50.0f;

                    chassisMsgPtr = &txChassisMessage;
                    xQueueSend(chassisMsgQueue, (void*)&chassisMsgPtr, (TickType_t)0);

                    lastChassisTime = currChassisTime;
                    currChassisTime = HAL_GetTick();
                    timeSinceLastChassisMessage = currChassisTime - lastChassisTime;
                }
                break;
            }
            case structSync: {
                lagTestEnd = HAL_GetTick();
                lagTestRoundTrip = lagTestEnd - lagTestStart;
                lagTestStart = HAL_GetTick();
                HAL_UART_Transmit(&huart8, (uint8_t*)lagTestArray, sizeof(lagTestArray), 50);
            }
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
