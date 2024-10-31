#pragma once

#include <subsystem.h>
#include <packet.h>
#include <BMI088.h>


class BMI088SubsystemClass : public TickableSubsystem, public DataProvider<SixFloats> {
public:
    BMI088SubsystemClass();
    virtual ~BMI088SubsystemClass();

    BaseSubsystem::Status setup();
    BaseSubsystem::Status tick();

private:
    Bmi088 bmi088;
    Bmi088Gyro gyro;
    Bmi088Accel accel;
    float temp;

};

extern BMI088SubsystemClass BMI088Subsystem;