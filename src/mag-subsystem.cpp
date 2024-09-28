#include "mag-subsystem.h"
#include "pins.h"
#include <Wire.h>
#include "log.h"

MagSubsystemClass MagSubsystem;

MagSubsystemClass::MagSubsystemClass() : DataThing<threeFloats>(rwLock) {
    name = "magenetometer subystem";
};

MagSubsystemClass::~MagSubsystemClass() {
}

BaseSubsystem::Status MagSubsystemClass::setup() {
    Wire.begin(I2C_SDA, I2C_SCL);
    setStatus(BaseSubsystem::FAULT);

    if (!sensor.begin_I2C()) {
        Log.errorln("Failed to find LIS3MDL chip");
        goto out;
    }
    sensor.setPerformanceMode(LIS3MDL_HIGHMODE);
    sensor.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    sensor.setDataRate(LIS3MDL_DATARATE_155_HZ); //uhh sure?
    sensor.setRange(LIS3MDL_RANGE_4_GAUSS);
    setStatus(BaseSubsystem::READY);

out:
    return getStatus();
}

BaseSubsystem::Status MagSubsystemClass::tick() {
    static sensors_event_t event;

    sensor.getEvent(&event);

    rwLock.Lock();
    data.x = event.magnetic.x;
    data.y = event.magnetic.y;
    data.z = event.magnetic.z;
    rwLock.UnLock();

    callCallbacks();
    return getStatus();
}
