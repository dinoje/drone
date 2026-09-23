#pragma once

#include <stdint.h>

struct MotorCommands {

    uint16_t m0;
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
};


MotorCommands mixQuadX(
    float baseThrottle,
    float rollOutput,
    float pitchOutput,
    float yawOutput
);