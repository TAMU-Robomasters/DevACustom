#pragma once
#include "information/can_protocol.hpp"

extern userCAN::device_t canDevice;
// CAN device object creation

namespace userInit {

extern void initialize(); // called in indicatorTaskFunc

} // namespace userInit
