#include "baro-subsystem.h"
#include "log.h"
#include "statusmanager.h"
#include "pins.h"

BaroSubsystemClass BaroSubystem;

BaroSubsystemClass::BaroSubsystemClass() :DataProvider<BarometerData>(rwLock), ms5611(0x76) {
    name = "baro";
    static BaseSubsystem* deps[] = {&StatusManager, &LogWriter, NULL};
    static SubsystemManagerClass::Spec spec(this, deps);
    SubsystemManager.addSubsystem(&spec);
}

BaroSubsystemClass::~BaroSubsystemClass() {
}

BaseSubsystem::Status BaroSubsystemClass::setup() {
    setStatus(BaseSubsystem::FAULT);
    Wire.begin(I2C_SDA, I2C_SCL);
    if (ms5611.begin()) {
        /*
        There are 5 oversampling settings, each corresponding to a different amount of milliseconds
        The higher the oversampling, the more accurate the reading will be, however the longer it will take.
        OSR_ULTRA_HIGH -> 8.22 millis
        OSR_HIGH       -> 4.11 millis
        OSR_STANDARD   -> 2.1 millis
        OSR_LOW        -> 1.1 millis
        OSR_ULTRA_LOW  -> 0.5 millis   Default = backwards compatible
        */
        ms5611.setOversampling(OSR_STANDARD); // should make 2.1 ms conversion
        ms5611.setCompensation(true);
        setStatus(BaseSubsystem::READY);
    }
out:
    return getStatus();
}

BaseSubsystem::Status BaroSubsystemClass::tick() {
    static constexpr float seaLevelhPa = 101325;
    float p; // in mBar/hPa

    rwLock.Lock();
    if (ms5611.read() != MS5611_READ_OK) {
        setStatus(BaseSubsystem::FAULT);
        goto out;
    }
    // FIXME: fixit
    // WTF gps: 75.478
    // baro: 25893
    // ooff by a factor of 343
    // see https://www.amsys-sensor.com/downloads/notes/ms5611-precise-altitude-measurement-with-a-pressure-sensor-module-amsys-509e.pdf
    p = ms5611.getPressure();
    data.temperature = ms5611.getTemperature();
    // formula courtesy of https://github.com/adafruit/Adafruit_DPS310/blob/master/Adafruit_DPS310.cpp#L271
    //data.altitude = 44330 * (1.0 - pow((p / 100) / seaLevelhPa, 0.1903));
    // from https://github.com/jarzebski/Arduino-MS5611/blob/dev/src/MS5611.cpp#L200
    data.altitude = (44330.0f * (1.0f - powf(p / seaLevelhPa, 0.1902949f)));

out:
    rwLock.UnLock();
    callCallbacks();
    return getStatus();
}

BarometerData BaroSubsystemClass::getBarometerData() const {
    rwLock.RLock();
    auto ret = data;
    rwLock.RUnlock();
    return ret;
}