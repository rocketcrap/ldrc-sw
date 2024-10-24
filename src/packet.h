#ifndef _PACKET_H
#define _PACKET_H

#include <stdint.h>
#include <ArduinoJson.h>
#include "configmanager.h"

static size_t constexpr PACKET_SIZE = 255;
size_t constexpr PACKET_HEADER_SIZE = 2+1+2+1+1;
static size_t constexpr PACKET_MESSAGE_SIZE = PACKET_SIZE - PACKET_HEADER_SIZE;


// normal packet: header|data
// relayed packet: header|(relay header|original header|original data)
static size_t constexpr RELAY_PACKET_HEADER_SIZE = 2+2+4;
static size_t constexpr MAX_MESSAGE_LEN = PACKET_MESSAGE_SIZE - PACKET_HEADER_SIZE - RELAY_PACKET_HEADER_SIZE;

struct __attribute__((packed)) Packet
{
    enum PacketTypes : uint8_t
    {
        DUMB_MESSAGE = 0,
        STATUS_MESSAGE = 1,
        EVENT_MESSAGE = 2,
        GPS_MESSAGE = 3,
        MINI_STATUS_MESSAGE = 4,
        NAME_MESSAGE = 5,
        RELAY_MESSAGE = 6,
    };
    enum Status : uint16_t
    {
        GPS_STATUS = (1 << 0),
        IMU_STATUS = (1 << 1),
        FLASH_STATUS = (1 << 2),
        RADIO_STATUS = (1 << 6),
        BATTERY_STATUS = (1 << 7),
        BAROMETER_STATUS = (1 << 10),
        LOGGER_STATUS = (1 << 11)
    };
    enum State : uint8_t
    {
        INIT,
        DISARMED,
        ARMED,
        BOOST,
        COAST,
        APOGEE,
        UNDER_CHUTE,
        LAWN_DART,
        TOUCHDOWN,
        LOST,
        POWER_FAIL,
        UNKNOWN
    };
    uint16_t sender; // CRC16 CCIT of ID
    uint8_t packetType;
    uint16_t packetCRC;
    uint8_t packetMessageLen;
    uint8_t packetMessage[PACKET_MESSAGE_SIZE];

    uint16_t calculateCRC() const;
    bool validateCRC() const;

    void setTypeSizeCRC(uint8_t packetType, size_t packetMessageLen);

    const uint8_t *asBytes() const;
    static Packet *fromBytes(uint8_t *bytes);
};

// Prolly will move to GPS eventually
struct __attribute__((packed)) GPSFix
{
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    uint32_t epoch;
    uint8_t fixType;
    uint8_t sats;

    static double degToDouble(int32_t deg);
};
bool convertToJson(const GPSFix& src, JsonVariant dst);

struct __attribute__((packed)) BarometerStatus {
    uint16_t altitude;
    uint8_t temperature;
};
bool convertToJson(const BarometerStatus& src, JsonVariant dst);

struct __attribute__((packed)) SixFloats
{
    float x;
    float y;
    float z;
    float pitch;
    float roll;
    float yaw;
};
bool convertToJson(const SixFloats& src, JsonVariant dst);

struct __attribute__((packed)) PyroStatus
{
    uint8_t continuity;

    void setContinuity(const int channel) noexcept;
    void clearContinuity(const int channel) noexcept;
};

struct __attribute__((packed)) Estimation
{
    SixFloats position;
    SixFloats velocity;
};
bool convertToJson(const Estimation& src, JsonVariant dst);


bool convertToJson(const Packet::State& src, JsonVariant dst);

struct __attribute__((packed)) MemoryStats {
    uint16_t kbAlloc;
    uint16_t kbFree;
    uint16_t storageTot;
    uint16_t storageUsed;
};
bool convertToJson(const MemoryStats& src, JsonVariant dst);


// 110 bytes
struct __attribute__((packed)) StatusPacket
{
    uint32_t timestamp; // millis() when this was generated
    GPSFix gpsFix;
    MemoryStats memoryStats;
    uint8_t batteryVoltage;
    BarometerStatus barometerData;
    SixFloats imuData;
    PyroStatus pyroStatus;
    Estimation estimate;
    Packet::State state;
    Packet::Status status;
};
bool convertToJson(const StatusPacket& src, JsonVariant dst);


struct __attribute__((packed)) DumbPacket
{
    char message[PACKET_MESSAGE_SIZE];
};


struct __attribute__((packed)) MinimalPacket
{
    uint32_t timestamp;
    GPSFix gpsFix;
    uint8_t batteryVoltage;
    Packet::State state;
    Packet::Status status;
};

struct __attribute__((packed)) GPSPacket
{
    GPSFix fix;
};

struct __attribute__((packed)) NamePacket {
    uint16_t sender;
    char name[32+1];
    ConfigData::Mode mode;
};
bool convertToJson(const NamePacket& src, JsonVariant dst);


struct __attribute__((packed)) RelayPacket {
    uint16_t originalSender;
    uint16_t receivedRSSI;
    uint32_t received;

    uint8_t packet[MAX_MESSAGE_LEN];
};

#endif //_PACKET_H