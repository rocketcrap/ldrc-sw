#include "eventmanager.h"

EventManagerClass EventManager;

EventManagerClass::EventManagerClass() {
    static BaseSubsystem* deps[] = {NULL};
    static SubsystemManagerClass::Spec spec(this, deps);

    SubsystemManager.addSubsystem(&spec);
    name = "eventManager";
    queue = xQueueCreateStatic(QUEUE_DEPTH,
        sizeof(Event),
        queueStorage,
        &staticQueue);
}
EventManagerClass::~EventManagerClass() {}

BaseSubsystem::Status EventManagerClass::setup() {
    setStatus(READY);
    return getStatus();
}
BaseSubsystem::Status EventManagerClass::start() {
    setStatus(RUNNING);
    return getStatus();
}

void EventManagerClass::subscribe(EventManagerClass::EventFn fn, uint32_t mask, void *ctx) {
    if (fn == NULL) {
        return;
    }

    rwLock.Lock();

    auto subscription = &subscriptions[numSubscriptions];
    // subscription cannot be NULL here, I think
    subscription->fn = fn;
    subscription->mask = mask;
    subscription->ctx = ctx;
    numSubscriptions++;

    rwLock.UnLock();
}

void EventManagerClass::taskFunction(void *parameter) {
    static Event event;
    while(1) {
        if (xQueueReceive(queue, &event, portMAX_DELAY) == pdPASS) {
            rwLock.RLock();
            for (auto i=0; i < numSubscriptions; i++) {
                auto subscription = &subscriptions[i];
                if (event.eventType & subscription->mask != 0) {
                    subscription->fn(event, subscription->ctx);
                }
            }
            rwLock.RUnlock();
        }
    }
}

void EventManagerClass::publishEvent(Event event) {
    event.timestamp = millis();
    xQueueSend(queue, &event, 0);
}
