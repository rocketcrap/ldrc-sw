#pragma once

#include <subsystem.h>
#include "packet.h"
#include "MS5611.h"

class BaroSubsystemClass : public TickableSubsystem, public DataThing<BarometerData> {
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