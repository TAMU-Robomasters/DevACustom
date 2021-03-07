#pragma once
#include "information/can_protocol.hpp"
#include "main.h"
#include "string.h"
#include "usart.h"
#include <memory>

const int max_serial_value_size = 1024;
const int serial_buffer_size = (max_serial_value_size * 2) * 2;

namespace userUART {

template <int maxSize, typename T = uint8_t>
class circularBuffer{

	private:
		T buffer[maxSize];
		int head = 0;
		int tail = 0;
    T delimiter;
    T* lastWord = NULL;
		// vector<T> lastWord;
		int lastWordSize = 0;

	public:
		circularBuffer<maxSize, T>(T delimiter = '\n') : delimiter(delimiter){};

		bool isFull(){
			return head == (tail + 1) % maxSize;
		}

		bool isEmpty(){
			return tail == head;
		}
		
		T front(){
			return buffer[head];
		}
		
		T get(size_t index){
			return buffer[index];
		}

		size_t getHead(){
			return head;
		}
		
		size_t getTail(){
			return tail;
		}
		
		size_t getSize(){
			if(tail >= head){
				return tail - head;
			}
			if(head > tail){
				return maxSize - tail - head;
			}
			return 0;
		}
		
		// vector<T> getLastWord(){
		//     return lastWord;
		// }
		
		T* getLastWord(){
			return lastWord;
		}
		
		size_t getLastWordSize(){
			return lastWordSize;
		}
		
		size_t getMaxSize(){
			return maxSize;
		}
		
		bool enqueue(T item){
			if(!isFull()){
				buffer[tail] = item;
				tail = (tail + 1) % maxSize;
			}
			if (item == delimiter){
				reset();
				return true;
			}
			return false;
		}
		
		T dequeue(){
			if (!isEmpty()){
				T item = buffer[head];
				T empty;
				buffer[head] = empty;
				head = (head + 1) % maxSize;
				return item;
			}
			return 0;
		}
		
		void reset(){
			delete[] lastWord;
			// lastWord.clear();
			int sz = getSize();
			//vector<T> wordation (sz);
			// lastWord = vector<T>(sz);
			// lastWord.reserve(sz);
			lastWord = new T[sz];
			// lastWordSize = sz;
			
			for (int i = 0; i < sz; i++)
			{
				lastWord[i] = dequeue();
				// lastWord.push_back(dequeue());
			}
			
			head = tail;
		}

};

extern circularBuffer<serial_buffer_size, uint8_t> serialBuffer6;

extern void motorFeedbackOut(UART_HandleTypeDef* huart, struct userCAN::motorFeedback_t* data);
// function used to output motor feedback info over UART

extern void task();
// called in freertos.cpp

extern void receive();
extern void send();
// !!!

} // namespace userUART
