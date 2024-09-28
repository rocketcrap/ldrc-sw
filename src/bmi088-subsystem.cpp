#include "bmi088-subsystem.h"
#include "pins.h"
#include "log.h"
#include <SPI.h>

BMI088SubsystemClass BMI088Subsystem;

BMI088SubsystemClass::BMI088SubsystemClass() :
    DataThing<SixFloats>(rwLock), bmi088(SPI, IMU_CSB1, IMU_CSB2),
    gyro(SPI, IMU_CSB2), accel(SPI, IMU_CSB1) {
    name = "BMI088 Subsystem";
    Bmi088(SPI, IMU_CSB1, IMU_CSB2);
}

BMI088SubsystemClass::~BMI088SubsystemClass() {

}

BaseSubsystem::Status BMI088SubsystemClass::setup() {
    int rc;

    setStatus(BaseSubsystem::FAULT);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    rc = accel.begin();
    if (rc < 0) {
        Log.errorln("bmi088 accel error: %d", rc);
        goto out;
    }
    rc = gyro.begin();
    if (rc < 0) {
        Log.errorln("bmi088 acceel error: %d", rc);
        goto out;
    }
    bmi088.setRange(Bmi088::ACCEL_RANGE_24G, Bmi088::GYRO_RANGE_1000DPS); // ?? idk


    // FIXME: read gets (according to docs- confirm in source) NEWEST data in buffer
    // we need to sample at very high frequency, but hw has a buffer of 64? entries?
    // so we need to sample theoritically 1/64 as fast. But really, a little faster

    // maaaybe interrupt???
    /* TODO: set some mode stuff */

    setStatus(BaseSubsystem::READY);
out:
    return getStatus();
}

BaseSubsystem::Status BMI088SubsystemClass::tick() {
    rwLock.Lock();

    bmi088.readSensor();

    data.x = accel.getAccelX_mss();
    data.y = accel.getAccelY_mss();
    data.z = accel.getAccelZ_mss();

    data.pitch = gyro.getGyroX_rads();
    data.roll = gyro.getGyroY_rads();
    data.yaw = gyro.getGyroZ_rads();

    temp = accel.getTemperature_C();

    rwLock.UnLock();

    callCallbacks();
    return getStatus();
}
