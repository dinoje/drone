#include "Mixer.h"

#include <Arduino.h>

#include "Config.h"


MotorCommands mixQuadX(
    float baseThrottle,
    float rollOutput,
    float pitchOutput,
    float yawOutput
)
{
    float m0 =
        baseThrottle +
        rollOutput +
        pitchOutput +
        yawOutput;

    float m1 =
        baseThrottle -
        rollOutput +
        pitchOutput -
        yawOutput;

    float m2 =
        baseThrottle +
        rollOutput -
        pitchOutput -
        yawOutput;

    float m3 =
        baseThrottle -
        rollOutput -
        pitchOutput +
        yawOutput;


    MotorCommands motors;

    motors.m0 =
        (uint16_t)constrain(
            (int)m0,
            DSHOT_MIN_THROTTLE,
            DSHOT_MAX_THROTTLE_90
        );

    motors.m1 =
        (uint16_t)constrain(
            (int)m1,
            DSHOT_MIN_THROTTLE,
            DSHOT_MAX_THROTTLE_90
        );

    motors.m2 =
        (uint16_t)constrain(
            (int)m2,
            DSHOT_MIN_THROTTLE,
            DSHOT_MAX_THROTTLE_90
        );

    motors.m3 =
        (uint16_t)constrain(
            (int)m3,
            DSHOT_MIN_THROTTLE,
            DSHOT_MAX_THROTTLE_90
        );

    return motors;
}