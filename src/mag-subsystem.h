#pragma once

#include <subsystem.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

struct threeFloats {
    float x;
    float y;
    float z;
};

class MagSubsystemClass : public TickableSubsystem, public DataProvider<threeFloats> {
public:
    MagSubsystemClass();
    virtual ~MagSubsystemClass();
    BaseSubsystem::Status setup();
    BaseSubsystem::Status tick();
    int period() const;

private:
    Adafruit_LIS3MDL sensor;
};

extern MagSubsystemClass MagSubsystem;