#pragma once

#include "motors/DShot.h"

class FlightController{

public:
    void begin();
    void update();

private:
    DShot dshot;

};