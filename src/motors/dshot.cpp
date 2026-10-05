#include "dshot.h"

#include <Arduino.h>
#include <driver/rmt.h>

#include "Config.h"
#include "Pins.h"

namespace {

rmt_channel_t rmtChannels[MOTOR_COUNT] = {
    RMT_CHANNEL_0,
    RMT_CHANNEL_1,
    RMT_CHANNEL_2,
    RMT_CHANNEL_3
};

rmt_item32_t dshotItems[MOTOR_COUNT][16];

}


// ============================================================
// INITIALIZATION
// ============================================================

void DShot::begin()
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        initMotor(i);
    }
}


void DShot::initMotor(uint8_t motorIndex)
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(
        (gpio_num_t)MOTOR_PINS[motorIndex],
        rmtChannels[motorIndex]
    );

    config.clk_div = 3;
    config.mem_block_num = 1;

    rmt_config(&config);

    rmt_driver_install(
        rmtChannels[motorIndex],
        0,
        0
    );
}


// ============================================================
// SEND DSHOT
// ============================================================

void DShot::send(
    uint8_t motorIndex,
    uint16_t value,
    bool telemetry
)
{
    value &= 0x7FF;

    uint16_t frame =
        (value << 1) |
        (telemetry ? 1 : 0);

    uint8_t crc =
        ((frame >> 0) ^
         (frame >> 4) ^
         (frame >> 8)) & 0x0F;

    frame =
        (frame << 4) |
        crc;


    for (int bit = 0; bit < 16; bit++) {

        bool one =
            frame & (1 << (15 - bit));

        dshotItems[motorIndex][bit].level0 = 1;

        dshotItems[motorIndex][bit].duration0 =
            one ? DSHOT_T1H : DSHOT_T0H;

        dshotItems[motorIndex][bit].level1 = 0;

        dshotItems[motorIndex][bit].duration1 =
            one ? DSHOT_T1L : DSHOT_T0L;
    }

    rmt_write_items(
        rmtChannels[motorIndex],
        dshotItems[motorIndex],
        16,
        true
    );
}


// ============================================================
// SEND SAME VALUE TO ALL MOTORS
// ============================================================

void DShot::sendAll(uint16_t value)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        send(i, value, false);
    }
}


// ============================================================
// ESC COMMAND
// ============================================================

void DShot::sendCommand(
    uint8_t motorIndex,
    uint16_t command,
    int times
)
{
    for (int i = 0; i < times; i++) {

        send(
            motorIndex,
            command,
            true
        );

        delay(1);
    }
}