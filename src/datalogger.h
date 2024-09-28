#pragma once

#include <subsystem.h>
#include "packet.h"
#include "eventmanager.h"
#include "statusmanager.h"
#include <LittleFS.h>


/**
 * @brief DataLogger logs key data in binary format to flash in regular intervals
 *
 * The DataLogger will essentailly log status packets every n milliseconds and
 * also record events
 *
 * The file backing will be managed such that it does not fill up the disk
 *
 * The tick should be called every 100ms or so in flight, but every 10s on pad and ground
 *
 *  let's imagine a 600 second flight w/ combined one hour on the ground
 *  assuming status packets are 128 bytes
 *  flight portion would take 750kB
 *  one hour combined pad and ground at 10s interval would take 45kB
 *
 *  all together that is approx 800kB/flight
 *     "storageTot": 1408,
 *     "storageUsed": 156
 *
 *  with change to partition, now we have:
 *     "storageTot": 1536,
 *     "storageUsed": 156
 *  that gives 1380KB free
 *
 *  we need to make flights smaller
 *
 *  30m on pad at 10s
 *  30s to apogee at 100ms
 *  300s to gnd at 1s
 *
 *
 *  okay, so the only way to adjust tick frequency means we need to
 *  make the adjustable ticking system
 *  which is a big change
 *
 * Fuck it, we make it a threaded subsystem and fixit later
 *
 *
 */
class DataLoggerClass : public ThreadedSubsystem {
    public:
        DataLoggerClass();
        virtual ~DataLoggerClass();
        virtual Status setup();
        virtual Status start();

        uint32_t startFlush, endFlush; // FIXME: temprorary

    protected:
        virtual void taskFunction(void *parameter);

    private:
        struct LogItem {
            enum ItemType : uint8_t {
                STATUS_ITEM = 0,
                EVENT_ITEM = 1,
                INVALID_ITEM = 0xFF
            } itemType;
            size_t itemSize;
            union {
                Event event;
                StatusPacket statusPacket;
            } item;
            const uint8_t *asBytes() const {
                return (const u_int8_t*)(&item);
            }
        };

        static constexpr size_t QUEUE_DEPTH = 8;
        static constexpr size_t BUFFER_SIZE = 4;

        uint8_t queueStorage[QUEUE_DEPTH * sizeof(Event)];
        QueueHandle_t queue;
        StaticQueue_t staticQueue;
        uint32_t periodMS;
        LogItem buffer[BUFFER_SIZE];
        size_t buffLen;
        fs::File file;

        void setPeriod(uint32_t period);
        uint32_t getPeriod() const;
        void LogEvent(const Event& event);
        void LogStatus(const StatusPacket& status);
        void setPeriodFromEvent(const Event& event);
};

extern DataLoggerClass DataLogger;