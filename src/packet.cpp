#include "packet.h"
#include <CRC16.h>

uint16_t Packet::calculateCRC() const
{
    CRC16 crc;

    crc.reset();
    crc.setPolynome(0x1021); // CCIT
    crc.add(packetType);
    crc.add(packetMessageLen);
    for (int i = 0; i < packetMessageLen; i++)
    {
        crc.add(packetMessage[i]);
    }
    return crc.calc();
}

bool Packet::validateCRC() const
{
    return (packetCRC == calculateCRC());
}

const uint8_t *Packet::asBytes() const
{
    auto self = (const uint8_t *)(this);
    return self;
}

Packet *Packet::fromBytes(uint8_t *bytes)
{
    Packet *self = (Packet *)bytes;
    return self;
}

void Packet::setTypeSizeCRC(uint8_t packetType, size_t packetMessageLen)
{
    this->packetType = packetType;
    this->packetMessageLen = packetMessageLen;
    this->packetCRC = calculateCRC();
}

void PyroStatus::setContinuity(const int channel) noexcept
{
    const uint8_t mask = (0x1 << channel);
    continuity |= mask;
}

void PyroStatus::clearContinuity(const int channel) noexcept
{
    const uint8_t mask = ~(0x1 << channel);
    continuity &= mask;
}

double GPSFix::degToDouble(int32_t deg) {
    // sparkfun gps lib returns long representing the number of degrees *10^-7
    return deg / 10000000.0;
}

bool convertToJson(const GPSFix& src, JsonVariant dst) {
    dst["sats"] = src.sats;
    dst["lat"] = GPSFix::degToDouble(src.latitude);
    dst["lng"] = GPSFix::degToDouble(src.longitude);
    dst["alt"] = src.altitude / 1000.0;
    dst["time"] = src.epoch;
    const char* fixType;
    // 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
    switch (src.fixType) {
        case 0:
            fixType = "nofix";
            break;
        case 1:
            fixType = "dead reckoning";
            break;
        case 2:
            fixType = "2D";
            break;
        case 3:
            fixType = "3D";
            break;
        case 4:
            fixType = "GNSS";
            break;
        case 5:
            fixType = "Time";
            break;
        default:
            fixType = "INVALID";
            break;
    }
    dst["fix"] = fixType;
    return true;
}

bool convertToJson(const Packet::State& src, JsonVariant dst) {
    const char* str;
    switch (src) {
        case Packet::INIT:
            str = "INIT";
            break;
        case Packet::ARMED:
            str = "ARMED";
            break;
        case Packet::BOOST:
            str = "BOOST";
            break;
        case Packet::COAST:
            str = "COAST";
            break;
        case Packet::APOGEE:
            str = "APOGEE";
            break;
        case Packet::UNDER_CHUTE:
            str = "UNDER_CHUTE";
            break;
        case Packet::LAWN_DART:
            str = "LAWN_DART";
            break;
        case Packet::TOUCHDOWN:
            str = "TOUCHDOWN";
            break;
        case Packet::POWER_FAIL:
            str = "POWER_FAIL";
            break;
        case Packet::UNKNOWN:
            str = "UNKNOWN";
            break;
        default:
            str = "INVALID";
            break;
    }
    dst.set(str);
    return true;
}


bool convertToJson(const MemoryStats& src, JsonVariant dst) {
    dst["kbAlloc"] = src.kbAlloc;
    dst["kbFree"] = src.kbFree;
    dst["storageTot"] = src.storageTot;
    dst["storageUsed"] = src.storageUsed;
    return true;
}

bool convertToJson(const StatusPacket& src, JsonVariant dst) {
    dst["type"] = "statusPacket";
    dst["timestamp"] = src.timestamp;
    dst["memStats"] = src.memoryStats;
    dst["batVolt"] = src.batteryVoltage / 10.0;
    dst["barometer"] = src.barometerData;
    dst["imu"] = src.imuData;
    dst["gps"] = src.gpsFix;
    // FIXME: pyrostatus
    dst["estimate"] = src.estimate;
    dst["state"] = src.state;
    //FIXME: status
    return true;
}


bool convertToJson(const SixFloats& src, JsonVariant dst) {
    dst["x"] = src.x;
    dst["y"] = src.y;
    dst["z"] = src.z;
    dst["pitch"] = src.pitch;
    dst["yaw"] = src.yaw;
    dst["roll"] = src.roll;
    return true;
}

bool convertToJson(const BarometerStatus& src, JsonVariant dst) {
    dst["altitude"] = src.altitude;
    dst["temp"] = src.temperature;
    return true;
}

bool convertToJson(const Estimation& src, JsonVariant dst) {
    dst["position"] = src.position;
    dst["velocity"] = src.velocity;
    return true;
}

bool convertToJson(const NamePacket& src, JsonVariant dst) {
    dst["type"] = "namePacket";
    dst["sender"] = src.sender;
    dst["name"].set(src.name);
    dst["mode"] = modeToString(src.mode);
    return true;
}
