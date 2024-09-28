#pragma once

#include <subsystem.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

struct threeFloats {
    float x;
    float y;
    float z;
};

class MagSubsystemClass : public TickableSubsystem, public DataThing<threeFloats> {
public:
    MagSubsystemClass();
    virtual ~MagSubsystemClass();
    BaseSubsystem::Status setup();
    BaseSubsystem::Status tick();

private:
    Adafruit_LIS3MDL sensor;
};

extern MagSubsystemClass MagSubsystem;