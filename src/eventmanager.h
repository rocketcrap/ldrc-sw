#pragma once

#include <subsystem.h>



// FIXME: This only considers rocket events. What are the other modalities of events?
//        Flight computer, relay, ground station

struct Event {

// TODO: evict event stuff from packet
/*
   events defined in packet.h
    enum EventNumber : uint8_t
    {
        INVALID_EVENT = 0,
        RADIO_FAST_MODE_EVENT,
        RADIO_SLOW_MODE_EVENT,
        BATTERY_LOW_EVENT,
        ERROR_EVENT,
        NONSENSE_EVENT,
        TOP_OPEN_EVENT,
        BOTTOM_OPEN_EVENT,
        PYRO_FIRE_EVENT,
        PYRO_FIRE_FAIL_EVENT,
        PYRO_CONT_LOSS_EVENT,
    };

*/

    uint32_t timestamp;
    enum EventType : uint32_t {
        START_EVENT =           0,
        ARM_EVENT =             1 << 0,
        DISARM_EVENT =          1 << 1,
        LIFTOFF_EVENT =         1 << 2,
        BURNOUT_EVENT =         1 << 3,
        AIRSTART_EVENT =        1 << 4,
        PYRO_FIRE_EVENT =       1 << 5,
        CONTINUITY_LOSS_EVENT = 1 << 6,
        APOGEE_EVENT =          1 << 7,
        LAWN_DART_EVENT =       1 << 8,
        LANDING_EVENT =         1 << 9,
        LOST_ROCKET_EVENT =     1 << 10,
        LOW_BATTERY_EVENT =     1 << 11,

        ALL_EVENT_MASK =        0xFFFFFFFF
    } eventType;
    union {
        struct intArgs_t {
            int32_t eventArg1;
            int32_t eventArg2;
            int32_t eventArg3;
        } intArgs;
        struct stringArgs_t {
            char msg[12];
        } stringArgs;
    } args;
};

/**
 * @brief EventManager gives a pub/sub mechanism for events
 *
 */
class EventManagerClass : public ThreadedSubsystem {
    public:
        typedef void(EventFn)(const Event& event, void *ctx);

        EventManagerClass();
        virtual ~EventManagerClass();
        virtual BaseSubsystem::Status setup();
        virtual BaseSubsystem::Status start();

        /**
         * @brief subscribe to an event
         *
         * Your function will be called from the event manager thead. Your function should not block.
         * It is not an error to publish with a handler
         *
         * @param fn a function to be called if event matches
         * @param mask a bitmask for eventType
         * @param ctx an opaque ptr to any ctx to invoke function with
         */
        void subscribe(EventFn fn, uint32_t mask, void *ctx);

        /**
         * @brief publish an event to subscribers
         *
         * publishing should not block, but if the queue is full it will not wait.
         *
         * @param event the event you wish to publish
         */
        void publishEvent(Event event);

        /**
         * @brief publish event with no args
         * 
         * @param eventType type of event to publish
         */
        void publishEvent(Event::EventType eventType);


    protected:
        virtual void taskFunction(void *parameter);

    private:
        // 32 should be enough for anyone, right
        static constexpr size_t MAX_SUBSCRIPTIONS = 32;
        static constexpr size_t QUEUE_DEPTH = 8;

        uint8_t queueStorage[QUEUE_DEPTH * sizeof(Event)];
        QueueHandle_t queue;
        StaticQueue_t staticQueue;

        struct Subscription {
            EventFn *fn;
            uint32_t mask;
            void *ctx;
        };
        size_t numSubscriptions;
        Subscription subscriptions[MAX_SUBSCRIPTIONS];
};

extern EventManagerClass EventManager;