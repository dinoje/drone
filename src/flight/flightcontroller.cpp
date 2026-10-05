#include <Arduino.h>
#include "FlightController.h"


void FlightController::begin()
{
    Serial.begin(115200);

    // Initialize motors
    dshot.begin();

    // Keep motors stopped
    dshot.sendAll(0);

    // Give the ESCs a continuous disarm signal
    Serial.println("Disarming ESCs...");

    unsigned long start = millis();

    while (millis() - start < 2000)
    {
        dshot.sendAll(0);
        delay(1);
    }

    Serial.println("ESCs ready.");
    
    Serial.println("Flight controller initialized.");
}

void FlightController::update()
{
    // Eventually this is where your main flight-control loop goes.

    // For now:
    dshot.sendAll(200);
}