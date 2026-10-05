#include <Arduino.h>
#include "flight/FlightController.h"

FlightController flightController;

void setup()
{
    flightController.begin();
}

void loop()
{
    flightController.update();
}