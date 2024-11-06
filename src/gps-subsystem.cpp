#include "gps-subsystem.h"
#include "log.h"
#include "statusmanager.h"
#include "pins.h"

GPSSubsystemClass GPSSubsystem;

GPSSubsystemClass::GPSSubsystemClass() : DataProvider<GPSFix>(rwLock), gpsLoopMillis(0), positioningMillis(0), noFixYet(true) {
    name = "GPS";
    static BaseSubsystem* deps[] = {&StatusManager, &LogWriter, NULL};
    static SubsystemManagerClass::Spec spec(this, deps);
    SubsystemManager.addSubsystem(&spec);
    data.fixType = 0; // invalid fix
}

GPSSubsystemClass::~GPSSubsystemClass() {
}

BaseSubsystem::Status GPSSubsystemClass::setup() {
    setStatus(BaseSubsystem::FAULT);

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!gps.begin()) {
        Log.errorln("GPS did not enumerate");
        goto out;
    }
    if (!gps.setMeasurementRate(100)) {
        Log.errorln("Couldn't set gps rate to 10hz");
        goto out;
    }
    gps.setI2CpollingWait(25);
    setStatus(BaseSubsystem::READY);

out:
    return getStatus();
}

GPSFix GPSSubsystemClass::getFix() const {
    rwLock.RLock();
    auto ret = data;
    rwLock.RUnlock();
    return ret;
}

BaseSubsystem::Status GPSSubsystemClass::tick() {
    rwLock.Lock();

    data.fixType = gps.getFixType();
    data.latitude = gps.getLatitude();
    data.longitude = gps.getLongitude();
    data.altitude = gps.getAltitudeMSL();
    data.epoch = gps.getUnixEpoch();
    data.sats = gps.getSIV();

    // If this is the first fix, and we've never had a fix- set the time
    if (data.fixType > 1 && noFixYet) {
        noFixYet = false;
        setRTC();
        // TODO: define event for time set established and emit it
    }

    rwLock.UnLock();
    callCallbacks();
    return getStatus();
}

bool GPSSubsystemClass::lowPowerMode() {
    setRTC();
    auto ret = gps.powerSaveMode();
    if (ret) {
        setStatus(STOPPED);
    }
    return ret;
}

void GPSSubsystemClass::setRTC() {
    if (data.fixType > 1) {
        auto milliseconds = gps.getMillisecond();
        struct timeval tv = {
            .tv_sec = static_cast<time_t>(data.epoch), // squash warning about narrowing uint32 to int32
            .tv_usec = milliseconds*1000
        };
        settimeofday(&tv, NULL);
        Log.infoln("Setting time at %d.%ld", tv.tv_sec, tv.tv_usec);
    }
}