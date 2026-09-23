#pragma once

#include <stdint.h>

// ============================================================
// DSHOT
// ============================================================

constexpr uint16_t DSHOT_DISARM = 0;

constexpr uint16_t DSHOT_MIN_THROTTLE = 48;
constexpr uint16_t DSHOT_MAX_THROTTLE = 2047;

constexpr uint16_t DSHOT_MAX_THROTTLE_90 =
    static_cast<uint16_t>(DSHOT_MAX_THROTTLE * 0.9f);

// DShot commands
constexpr uint16_t DSHOT_CMD_SPIN_DIRECTION_1 = 20;
constexpr uint16_t DSHOT_CMD_SPIN_DIRECTION_2 = 21;
constexpr uint16_t DSHOT_CMD_SAVE_SETTINGS = 12;

// RMT timing
constexpr int DSHOT_T1H = 62;
constexpr int DSHOT_T1L = 26;
constexpr int DSHOT_T0H = 26;
constexpr int DSHOT_T0L = 62;


// ============================================================
// SBUS
// ============================================================

constexpr long SBUS_BAUDRATE = 100000;

constexpr int SBUS_FRAME_SIZE = 25;

constexpr uint8_t SBUS_HEADER = 0x0F;
constexpr uint8_t SBUS_FOOTER = 0x00;

// Throttle
constexpr uint16_t SBUS_THR_MIN = 1000;
constexpr uint16_t SBUS_THR_MAX = 1833;

constexpr uint16_t SBUS_THR_ARM_THRESHOLD =
    SBUS_THR_MIN + 20;

// Sticks
constexpr int SBUS_STICK_MIN = 167;
constexpr int SBUS_STICK_MID = 1000;
constexpr int SBUS_STICK_MAX = 1833;

constexpr int SBUS_DEADZONE = 20;

// Channels
constexpr int CH_ROLL = 0;
constexpr int CH_PITCH = 1;
constexpr int CH_THROTTLE = 2;
constexpr int CH_YAW = 3;


// ============================================================
// FLIGHT CONTROL
// ============================================================

constexpr float MAX_ANGLE_DEG = 25.0f;
constexpr float MAX_YAW_RATE = 120.0f;

// Complementary filter
constexpr float ALPHA = 0.97f;

// Gyro filtering
constexpr float GYRO_FILTER_OLD = 0.7f;
constexpr float GYRO_FILTER_NEW = 0.3f;

// Main loop
constexpr unsigned long CONTROL_PERIOD_US = 5000;