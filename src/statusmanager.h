#ifndef _STATUS_MANAGER_H
#define _STATUS_MANAGER_H

#include "packet.h"
#include "subsystem.h"

/**
 * @brief Status Manager maintains a status packet, which is is sent regularly over telemetry
 *
 * The abstraction of what is the vehicle status and what is the radio representation is pretty much 1:1 right now, but
 * in the future things like the estimate of position will be handled by an estimate manager and the represtation here
 * will be just the radio format
 *
 */
class StatusManagerClass : public TickableSubsystem, public DataThing<StatusPacket>
{
public:
    StatusManagerClass();
    virtual ~StatusManagerClass();
    virtual Status setup();
    virtual Status tick();

    MinimalPacket getMinimalPacket() const;

    PyroStatus getPyroStatus() const;

    void setGPSFix(const GPSFix &fix);
    void setMemoryStats(const MemoryStats &stats);
    void setBatteryVoltage(const uint8_t voltage);
    void setBarometerData(const BarometerData &barometerData);
    void setIMUData(const SixFloats &sixFloats);
    void setEstimate(const Estimation &estimate);
    void setPyroStatus(const PyroStatus &pyroStatus);
    void setState(const Packet::State state);
    void setStatus(const Packet::Status status); //FIXME: perhaps not useful
    void setStatusFlag(const Packet::Status status);
    void clearStatusFlag(const Packet::Status status);
    void setBoolStatusFlag(const bool value, const Packet::Status status);

private:
    void updateTimestamp();
};

extern StatusManagerClass StatusManager;

#endif // _STATUS_MANAGER_H