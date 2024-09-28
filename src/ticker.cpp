#include "ticker.h"

Ticker::Ticker(TickableSubsystem** _subsystems, int _intervalMS) : subsystems(_subsystems), intervalMS(_intervalMS) {
    name = "ticker";

    // Ticker is not a singleton and you should start/setup manually
    // static BaseSubsystem* deps[] = {NULL};
    // static SubsystemManagerClass::Spec spec = {
    //     .subsystem = this,
    //     .deps = deps,
    // };
    // SubsystemManager.addSubsystem(&spec);
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

int Ticker::taskPriority() const {
    return 1;
}

void Ticker::taskFunction(void *parameter) {
    const auto period = (intervalMS * configTICK_RATE_HZ) / 1000L;

    // Initialise with the current time.
    auto lastWakeTime = xTaskGetTickCount();
    while(1) {
        // Wait for the next cycle.
        vTaskDelayUntil(&lastWakeTime, period);

        for (auto i = 0; subsystems && subsystems[i] != nullptr; i++) {
            if (subsystems[i]->getStatus() != FAULT) {
                subsystems[i]->tick();
            }
        }
    }
}
