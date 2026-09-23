#include "SBUS.h"

#include <Arduino.h>

#include "Config.h"
#include "Pins.h"


void SBUS::begin()
{
    Serial2.begin(
        SBUS_BAUDRATE,
        SERIAL_8E2,
        SBUS_RX_PIN,
        -1,
        true
    );
}


bool SBUS::read()
{
    static uint8_t buffer[SBUS_FRAME_SIZE];
    static int index = 0;

    while (Serial2.available()) {

        uint8_t b = Serial2.read();

        // Wait for frame header
        if (index == 0 &&
            b != SBUS_HEADER) {
            continue;
        }

        buffer[index++] = b;

        if (index == SBUS_FRAME_SIZE) {

            index = 0;

            if (buffer[24] != SBUS_FOOTER) {
                return false;
            }

            bool frameLost =
                buffer[23] & (1 << 2);

            bool failsafe =
                buffer[23] & (1 << 3);

            if (frameLost || failsafe) {

                channels[CH_THROTTLE] = 0;

                return true;
            }


            channels[0] =
                ((uint16_t)buffer[1] |
                ((uint16_t)buffer[2] << 8))
                & 0x07FF;

            channels[1] =
                (((uint16_t)buffer[2] >> 3) |
                ((uint16_t)buffer[3] << 5))
                & 0x07FF;

            channels[2] =
                (((uint16_t)buffer[3] >> 6) |
                ((uint16_t)buffer[4] << 2) |
                ((uint16_t)buffer[5] << 10))
                & 0x07FF;

            channels[3] =
                (((uint16_t)buffer[5] >> 1) |
                ((uint16_t)buffer[6] << 7))
                & 0x07FF;

            channels[4] =
                (((uint16_t)buffer[6] >> 4) |
                ((uint16_t)buffer[7] << 4))
                & 0x07FF;

            channels[5] =
                (((uint16_t)buffer[7] >> 7) |
                ((uint16_t)buffer[8] << 1) |
                ((uint16_t)buffer[9] << 9))
                & 0x07FF;

            channels[6] =
                (((uint16_t)buffer[9] >> 2) |
                ((uint16_t)buffer[10] << 6))
                & 0x07FF;

            channels[7] =
                (((uint16_t)buffer[10] >> 5) |
                ((uint16_t)buffer[11] << 3))
                & 0x07FF;

            channels[8] =
                ((uint16_t)buffer[12] |
                ((uint16_t)buffer[13] << 8))
                & 0x07FF;

            channels[9] =
                (((uint16_t)buffer[13] >> 3) |
                ((uint16_t)buffer[14] << 5))
                & 0x07FF;

            channels[10] =
                (((uint16_t)buffer[14] >> 6) |
                ((uint16_t)buffer[15] << 2) |
                ((uint16_t)buffer[16] << 10))
                & 0x07FF;

            channels[11] =
                (((uint16_t)buffer[16] >> 1) |
                ((uint16_t)buffer[17] << 7))
                & 0x07FF;

            channels[12] =
                (((uint16_t)buffer[17] >> 4) |
                ((uint16_t)buffer[18] << 4))
                & 0x07FF;

            channels[13] =
                (((uint16_t)buffer[18] >> 7) |
                ((uint16_t)buffer[19] << 1) |
                ((uint16_t)buffer[20] << 9))
                & 0x07FF;

            channels[14] =
                (((uint16_t)buffer[20] >> 2) |
                ((uint16_t)buffer[21] << 6))
                & 0x07FF;

            channels[15] =
                (((uint16_t)buffer[21] >> 5) |
                ((uint16_t)buffer[22] << 3))
                & 0x07FF;

            return true;
        }
    }

    return false;
}


uint16_t SBUS::getChannel(uint8_t channel) const
{
    if (channel >= 16) {
        return 0;
    }

    return channels[channel];
}


uint16_t SBUS::throttleToDshot(
    uint16_t value
) const
{
    if (value == 0 ||
        value <= SBUS_THR_ARM_THRESHOLD) {

        return DSHOT_DISARM;
    }

    uint16_t result =
        (uint16_t)map(
            (long)value,
            SBUS_THR_MIN,
            SBUS_THR_MAX,
            DSHOT_MIN_THROTTLE,
            DSHOT_MAX_THROTTLE_90
        );

    return constrain(
        result,
        DSHOT_MIN_THROTTLE,
        DSHOT_MAX_THROTTLE_90
    );
}


float SBUS::stickToFloat(
    uint16_t value
) const
{
    int centered =
        (int)value - SBUS_STICK_MID;

    if (abs(centered) <
        SBUS_DEADZONE) {

        return 0.0f;
    }

    float range =
        centered > 0
        ? (float)(
            SBUS_STICK_MAX -
            SBUS_STICK_MID -
            SBUS_DEADZONE)

        : (float)(
            SBUS_STICK_MID -
            SBUS_STICK_MIN -
            SBUS_DEADZONE);

    float scaled =
        (float)(
            abs(centered) -
            SBUS_DEADZONE) / range;

    return constrain(
        scaled *
        (centered > 0 ? 1.0f : -1.0f),
        -1.0f,
        1.0f
    );
}