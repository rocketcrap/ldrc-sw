#include "datalogger.h"
#include "log.h"
#include "configmanager.h"

#define SECOND 10000
#define MINUTE (60*SECOND)

DataLoggerClass DataLogger;

DataLoggerClass::DataLoggerClass() : buffLen(0) {
    static BaseSubsystem* deps[] = {&StatusManager, &LogWriter, &ConfigManager, NULL};
    static SubsystemManagerClass::Spec spec(this, deps);
    SubsystemManager.addSubsystem(&spec);

    name = "DataLogger";
    queue = xQueueCreateStatic(QUEUE_DEPTH,
        sizeof(DataLoggerClass::LogItem),
        queueStorage,
        &staticQueue);
}

DataLoggerClass::~DataLoggerClass() {
}

BaseSubsystem::Status DataLoggerClass::setup() {
    EventManager.subscribe([](const Event& event, void *ctx) {
        auto self = static_cast<DataLoggerClass*>(ctx);
        self->LogEvent(event);
    },  0xFFFFFFFF, this);

    // TODO: file naming and cleanup by unix epoch time
    // if you do that, you can't setup files until you get the firstfix event
    // if you go ordinal names, you don't need to wait.

    // we can have up to n flight logs - perhaps 2
    // we should have a directory we put them
    // such as datalogs/datalog-%d - where %d is the unix epoch time
    // if storage is less than threshold- we clear the oldest
    // this requires that we actually only start when we know what time it is
    // either because the rtc is always set or the gps has a fix
    //
    // for now, just use the same file every time and clear it when rebooted

    file = LittleFS.open("/datalogs/datalog", "w");
    if (!file) {
        setStatus(BaseSubsystem::FAULT);
    } else {
        setStatus(BaseSubsystem::READY);
    }

    return getStatus();
}

BaseSubsystem::Status DataLoggerClass::start() {
    setStatus(BaseSubsystem::RUNNING);
    return getStatus();
}

void DataLoggerClass::taskFunction(void *paremeter) {
    while(1) {
        auto xLastWakeTime = xTaskGetTickCount();

        // this is a little confusing, but actually shove a status packet if our period is "on"
        if (getPeriod() > 0) {
            StatusManager.readData([](const StatusPacket &packet, void *ctx) {
                auto self = static_cast<DataLoggerClass*>(ctx);
                self->LogStatus(packet);
            }, this);
        } else {
            delay(SECOND);
            continue;
        }
        auto item = &(buffer[buffLen]);
        if (xQueueReceive(queue, item, SECOND) == pdPASS) {
            buffLen++;
            // if you just grabbed an event -or- buffer is full, flush
            if (item->itemType == LogItem::EVENT_ITEM) {
                setPeriodFromEvent(item->item.event);
            }
            if (item->itemType == LogItem::EVENT_ITEM || buffLen >= BUFFER_SIZE) {
                startFlush = millis();
                for (auto i = 0; i < buffLen; i++) {
                    const auto& logItem = buffer[i];
                    const auto size = logItem.itemSize;

                    file.write(logItem.itemType);
                    file.write(size);
                    file.write(logItem.asBytes(), size);
                }
                endFlush = millis();
            }
            buffLen = 0; // clear the buffer
        }
        vTaskDelayUntil(&xLastWakeTime, getPeriod());
    }
}

void DataLoggerClass::setPeriodFromEvent(const Event& event) {
    switch(event.eventType) {
        case Event::START_EVENT:
            setPeriod(0);
            break;
        case Event::ARM_EVENT:
            setPeriod(10*SECOND);
            break;
        case Event::DISARM_EVENT:
            setPeriod(0);
            break;
        case Event::LIFTOFF_EVENT:
            setPeriod(100);
            break;
        case Event::APOGEE_EVENT:
            setPeriod(1*SECOND);
            break;
        case Event::LANDING_EVENT:
            setPeriod(5*MINUTE);
            break;
        case Event::LOW_BATTERY_EVENT:
            setPeriod(0);
            break;
    }
}

void DataLoggerClass::setPeriod(uint32_t period) {
    rwLock.Lock();
    periodMS = period;
    rwLock.UnLock();
}

uint32_t DataLoggerClass::getPeriod() const {
    rwLock.RLock();
    auto ret = periodMS;
    rwLock.RUnlock();
    return ret;
}

void DataLoggerClass::LogEvent(const Event& event) {
    static LogItem item; // this is not a race condition- LogEvent is only called from the EventManager thread
    item.itemType = LogItem::EVENT_ITEM;
    item.item.event = event;
    item.itemSize = sizeof(Event);
    xQueueSend(queue, &item, 0);
}

void DataLoggerClass::LogStatus(const StatusPacket &status) {
    static LogItem item; // this is not a race condition - LogStatus is only called from this thread
    item.itemType = LogItem::STATUS_ITEM;
    item.item.statusPacket = status;
    item.itemSize = sizeof(StatusPacket);
    xQueueSend(queue, &item, 0);
}