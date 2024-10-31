#pragma once

#include <subsystem.h>
#include "MS5611.h"
#include "packet.h"

struct BarometerData {
    operator BarometerStatus() const {
        BarometerStatus status;
        status.altitude = roundf(altitude);
        status.temperature = temperature;
        return status;
    }
    float altitude;
    uint8_t temperature;
};


class BaroSubsystemClass : public TickableSubsystem, public DataProvider<BarometerData> {
public:
    BaroSubsystemClass();
    virtual ~BaroSubsystemClass();

    BaseSubsystem::Status setup();
    BaseSubsystem::Status tick();

    BarometerData getBarometerData() const;

private:
    MS5611 ms5611;
};

extern BaroSubsystemClass BaroSubystem;