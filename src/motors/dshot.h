#pragma once

#include <stdint.h>

class DShot {
public:
    void begin();

    void send(
        uint8_t motorIndex,
        uint16_t value,
        bool telemetry = false
    );

    void sendAll(uint16_t value);

    void sendCommand(
        uint8_t motorIndex,
        uint16_t command,
        int times
    );

private:
    void initMotor(uint8_t motorIndex);
};