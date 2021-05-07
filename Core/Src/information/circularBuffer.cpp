#include "information/uart_protocol.hpp"

using namespace userUART;

//circularBuffer<1024, uint8_t> dummy;

template <int maxSize, typename T>
bool circularBuffer<maxSize, T>::isFull() {
    return head == (tail + 1) % maxSize;
}

template <int maxSize, typename T>
bool circularBuffer<maxSize, T>::isEmpty() {
    return tail == head;
}

template <int maxSize, typename T>
T circularBuffer<maxSize, T>::front() {
    return buffer[head];
}

template <int maxSize, typename T>
T circularBuffer<maxSize, T>::get(size_t index) {
    return buffer[index];
}

template <int maxSize, typename T>
size_t circularBuffer<maxSize, T>::getHead() {
    return head;
}

template <int maxSize, typename T>
size_t circularBuffer<maxSize, T>::getTail() {
    return tail;
}

template <int maxSize, typename T>
size_t circularBuffer<maxSize, T>::getSize() {
    if (tail >= head) {
        return tail - head;
    }
    if (head > tail) {
        return maxSize - head + tail;
    }
    return 0;
}

// vector<T> getLastWord(){
//     return lastWord;
// }

template <int maxSize, typename T>
T* circularBuffer<maxSize, T>::getLastWord() {
    return lastWord;
}

template <int maxSize, typename T>
size_t circularBuffer<maxSize, T>::getLastWordSize() {
    return lastWordSize;
}

template <int maxSize, typename T>
size_t circularBuffer<maxSize, T>::getMaxSize() {
    return maxSize;
}

template <int maxSize, typename T>
bool circularBuffer<maxSize, T>::enqueue(T item) {
    if (item == delimiter) {
        reset();
        return true;
    }
    if (isFull()) {
        return true;
    }
    buffer[tail] = item;
    tail = (tail + 1) % maxSize;
    return false;
}

template <int maxSize, typename T>
T circularBuffer<maxSize, T>::dequeue() {
    if (isEmpty()) {
        return 0;
    }
    T item = buffer[head];
    // T empty;
    // buffer[head] = empty;
    head = (head + 1) % maxSize;
    return item;
}

template <int maxSize, typename T>
void circularBuffer<maxSize, T>::reset() {
    // lastWord.clear();
    size_t sz = getSize();
    //vector<T> wordation (sz);
    // lastWord = vector<T>(sz);
    // lastWord.reserve(sz);
    if (lastWord != NULL)
        delete[] lastWord;
    lastWord = new T[sz];
    lastWordSize = sz;
    for (size_t i = 0; i < sz; i++) {
        lastWord[i] = dequeue();
        // lastWord.push_back(dequeue());
    }

    // head = tail;
}

template <int maxSize, typename T>
circularBuffer<maxSize, T>::~circularBuffer() {
    delete[] lastWord;
}
