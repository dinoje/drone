#pragma once

#include <stdint.h>

class SBUS {
public:
    void begin();

    bool read();

    uint16_t getChannel(uint8_t channel) const;

    uint16_t throttleToDshot(uint16_t value) const;

    float stickToFloat(uint16_t value) const;

private:
    uint16_t channels[16] = {};
};