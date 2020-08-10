#pragma once
#include "information/can_protocol.hpp"
#include "main.h"
#include "string.h"
#include "usart.h"

/* 
	Our custom functions for UART/USART; heavily inspired by QDU
*/

namespace userUART {

extern void numOut(UART_HandleTypeDef* huart, uint8_t num);
// helper function that might be useful later

extern void stringOut(UART_HandleTypeDef* huart, char buffer[]);
// helper function that might be useful later

extern void newlnOut(UART_HandleTypeDef* huart);
// helper function that might be useful later

extern void motorfeedbackOut(UART_HandleTypeDef* huart, struct userCAN::motorFeedback_t* data);
// function used to output motor feedback info over UART

extern void task();
// called in freertos.cpp

extern void receive();
extern void send();
// !!!

} // namespace userUART
