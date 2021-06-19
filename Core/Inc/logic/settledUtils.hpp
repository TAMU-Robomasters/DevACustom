#pragma once

typedef enum {
    position,
    velocity,
} settleType;

extern bool isSettled(settleType st, float curr, float target, float tolerance);

extern bool waitUntilSettled(settleType st, float curr, float target, float tolerance);
