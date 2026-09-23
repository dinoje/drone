#pragma once

// ==============================
// MOTOR PINS
// ==============================

constexpr int MOTOR_COUNT = 4;

constexpr int MOTOR_PINS[MOTOR_COUNT] = {
    13, // M0 - Front Left
    14, // M1 - Front Right
    27, // M2 - Back Left
    26  // M3 - Back Right
};

// ==============================
// RECEIVER
// ==============================

constexpr int SBUS_RX_PIN = 16;