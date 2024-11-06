#include "ticker.h"

Ticker::Ticker(TickableSubsystem** _subsystems, int _intervalMS, const char *name, int priority) :
    subsystems(_subsystems), intervalMS(_intervalMS), priority(priority),
    spec(static_cast<BaseSubsystem*>(this), static_cast<BaseSubsystem**>(deps)) {
    this->name = name;

    bzero(deps, sizeof(deps)); // all values null
    for (auto i = 0; subsystems[i] && i < MAX_DEPS; i++) {
        deps[i] = subsystems[i];
    }
    SubsystemManager.addSubsystem(&spec);
}

BaseSubsystem::Status Ticker::setup() {
    BaseSubsystem::Status newStatus = FAULT;

    if (subsystems != nullptr) {
        newStatus = READY;
        for (auto i = 0; subsystems[i] != nullptr; i++) {
            subsystems[i]->setup();
        }
    }

    setStatus(newStatus);
    return newStatus;
}

int Ticker::period() const {
    rwLock.RLock();
    auto rc = intervalMS;
    rwLock.RUnlock();
    return rc;
}

void Ticker::setPeriod(int period) {
    rwLock.Lock();
    intervalMS = period;
    rwLock.UnLock();
}

int Ticker::taskPriority() const {
    return priority;
}

int Ticker::getPercentBusy() const {
    rwLock.RLock();
    auto rc = roundf((100 * medianDuration) / (intervalMS * 1000.0f));
    rwLock.RUnlock();
    return rc;
}

void Ticker::taskFunction(void *parameter) {
    // Initialise with the current time.
    auto lastWakeTime = xTaskGetTickCount();
    while(1) {
        const auto start = micros();

        for (auto i = 0; subsystems && subsystems[i] != nullptr; i++) {
            if (subsystems[i]->getStatus() == RUNNING) {
                subsystems[i]->tick();
            }
        }

        const auto end = micros();
        rwLock.Lock();
        medianDuration = durationFilter(end - start);
        rwLock.UnLock();

        // Wait for the next cycle.
        vTaskDelayUntil(&lastWakeTime, period());    }
}

bool Ticker::lowPowerMode() {
    for (auto i = 0; subsystems && subsystems[i] != nullptr; i++) {
        subsystems[i]->lowPowerMode();
    }
    return stop();
}

BaseSubsystem::Status Ticker::start() {
    if (getStatus() == STOPPED) {
        for (auto i = 0; subsystems && subsystems[i] != nullptr; i++) {
            subsystems[i]->start();
        }
    }
    return ThreadedSubsystem::start();
}
