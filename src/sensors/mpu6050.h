#pragma once

#include <Adafruit_MPU6050.h>

struct IMUData {

    float gyroX;
    float gyroY;
    float gyroZ;

    float accelX;
    float accelY;
    float accelZ;

    float temperature;
};


class MPU6050Sensor {

public:

    bool begin();

    bool read(IMUData& data);

    void calibrate();


private:

    Adafruit_MPU6050 mpu;

    float gyroXOffset = 0.0f;
    float gyroYOffset = 0.0f;
    float gyroZOffset = 0.0f;

    float accXOffset = 0.0f;
    float accYOffset = 0.0f;
};