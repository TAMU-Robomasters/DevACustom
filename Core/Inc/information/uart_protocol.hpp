#pragma once
#include "information/can_protocol.hpp"
#include "main.h"
#include "string.h"
#include "subsystems/chassis.hpp"
#include "subsystems/gimbal.hpp"
#include "usart.h"
#include <memory>

const int max_serial_value_size = 1024;
const int serial_buffer_size = (max_serial_value_size * 2) + 2;

namespace userUART {

extern QueueHandle_t aimMsgQueue;
extern QueueHandle_t gimbMsgQueue;
extern QueueHandle_t chassisMsgQueue;

struct aimMsgStruct {
    uint8_t prefix;
    float disp[2];
};

struct gimbMsgStruct {
    uint8_t prefix;
    gimbal::gimbalStates state;
    float yaw;
    // float pitch;
};

struct chassisMsgStruct {
    uint8_t prefix;
    chassis::chassisStates state;
    float m1;
    float m2;
    float m3;
    float m4;
};

typedef enum {
    aimAt = 'a',
    structSync = 's',
} jetsonMsgTypes;

typedef enum {
    gimbal = 'g',
    chassis = 'c',
} d2dMsgTypes;

template <int maxSize, typename T = uint8_t>
class circularBuffer{

	private:
		T buffer[maxSize];
        int head = 0;
        int tail = 0;
        T delimiter;
        T* lastWord = nullptr;
        // vector<T> lastWord;
		int lastWordSize = 0;

	public:
		circularBuffer<maxSize, T>(T delimiter = '\n') : delimiter(delimiter){};
		~circularBuffer();

		bool isFull();
		bool isEmpty();
		T front();
		T get(size_t index);
		size_t getHead();
		size_t getTail();
		size_t getSize();
		T* getLastWord();
		size_t getLastWordSize();
		size_t getMaxSize();
		bool enqueue(T item);
		T dequeue();
        void reset();
};

extern circularBuffer<serial_buffer_size, uint8_t> jetsonBuffer;
extern circularBuffer<serial_buffer_size, uint8_t> d2dBuffer;

extern void motorFeedbackOut(UART_HandleTypeDef* huart, struct userCAN::motorFeedback_t* data);
// function used to output motor feedback info over UART

extern void task();
// called in freertos.cpp

extern void receive();
extern void send();
// !!!

} // namespace userUART
