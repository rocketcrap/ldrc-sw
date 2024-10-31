#include "statusmanager.h"
#include <LittleFS.h>
#include "pins.h"

StatusManagerClass StatusManager;

StatusManagerClass::StatusManagerClass() : DataProvider<StatusPacket>(rwLock) {
    static BaseSubsystem* deps[] = {NULL};
    static SubsystemManagerClass::Spec spec(this, deps);

    SubsystemManager.addSubsystem(&spec);
    name = "statusManager";
}

StatusManagerClass::~StatusManagerClass() {
}

 BaseSubsystem::Status StatusManagerClass::setup() {
    pinMode(VDC, ANALOG);
    // name collision between setting the status packet status and base susbsystem
    BaseSubsystem::setStatus(READY);
    return getStatus();
 }

 BaseSubsystem::Status StatusManagerClass::tick() {
    static constexpr auto scale_factor = 110.0/10.0;

    // Update memory statistics
    MemoryStats memStats;
    multi_heap_info_t info;

    heap_caps_get_info(&info, MALLOC_CAP_INTERNAL);

    memStats.kbFree = info.total_free_bytes / 1024;
    memStats.kbAlloc = info.total_allocated_bytes / 1024;

    memStats.storageTot = LittleFS.totalBytes() / 1024;
    memStats.storageUsed = LittleFS.usedBytes() / 1024;
    setMemoryStats(memStats);

    auto volts = analogReadMilliVolts(VDC) / 1000.0 * scale_factor;
    setBatteryVoltage(volts * 10.0);
    return BaseSubsystem::RUNNING;
 }

MinimalPacket StatusManagerClass::getMinimalPacket() const {
    MinimalPacket ret;

    rwLock.RLock();
    ret.timestamp = data.timestamp;
    ret.state = data.state;
    ret.status = data.status;
    ret.batteryVoltage = data.batteryVoltage;
    ret.gpsFix = data.gpsFix;
    rwLock.RUnlock();

    return ret;
}

PyroStatus StatusManagerClass::getPyroStatus() const {
    rwLock.RLock();
    auto ret = data.pyroStatus;
    rwLock.RUnlock();
    return ret;
}


void StatusManagerClass::setGPSFix(const GPSFix &fix) {
    rwLock.Lock();
    data.gpsFix = fix;
    updateTimestamp();
    rwLock.UnLock();
}

void StatusManagerClass::setMemoryStats(const MemoryStats &stats) {
    rwLock.Lock();
    data.memoryStats = stats;
    updateTimestamp();
    rwLock.UnLock();
}

void StatusManagerClass::setBatteryVoltage(const uint8_t voltage) {
    rwLock.Lock();
    data.batteryVoltage = voltage;
    updateTimestamp();
    rwLock.UnLock();
}

void StatusManagerClass::setBarometerData(const BarometerStatus &barometerData) {
    rwLock.Lock();
    data.barometerData = barometerData;
    updateTimestamp();
    rwLock.UnLock();
}

void StatusManagerClass::setIMUData(const SixFloats &sixFloats) {
    rwLock.Lock();
    data.imuData = sixFloats;
    updateTimestamp();
    rwLock.UnLock();
}

void StatusManagerClass::setEstimate(const Estimation &estimate) {
    rwLock.Lock();
    data.estimate = estimate;
    updateTimestamp();
    rwLock.UnLock();
}

void StatusManagerClass::setPyroStatus(const PyroStatus &pyroStatus) {
    rwLock.Lock();
    data.pyroStatus = pyroStatus;
    updateTimestamp();
    rwLock.UnLock();
}

void StatusManagerClass::setState(const Packet::State state) {
    rwLock.Lock();
    data.state = state;
    updateTimestamp();
    rwLock.UnLock();
}

void StatusManagerClass::setStatus(const Packet::Status status) {
    rwLock.Lock();
    data.status = status;
    updateTimestamp();
    rwLock.UnLock();
}

void StatusManagerClass::setStatusFlag(const Packet::Status status) {
    rwLock.Lock();
    data.status = (Packet::Status)(data.status | status);
    updateTimestamp();
    rwLock.UnLock();
}

void StatusManagerClass::clearStatusFlag(const Packet::Status status) {
    rwLock.Lock();
    const auto inverted = ~status;
    data.status = (Packet::Status)(data.status & inverted);
    updateTimestamp();
    rwLock.UnLock();
}

void StatusManagerClass::setBoolStatusFlag(const bool value, const Packet::Status status) {
    if (value) {
        StatusManager.setStatusFlag(status);
    } else {
        StatusManager.clearStatusFlag(status);
    }
}

void StatusManagerClass::updateTimestamp() {
    data.timestamp = millis();
}
