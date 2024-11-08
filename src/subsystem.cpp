#include "subsystem.h"
#include <Arduino.h>


BaseSubsystem::BaseSubsystem() : status(BaseSubsystem::INIT), name("UNSET") {
}

BaseSubsystem::~BaseSubsystem() {}

BaseSubsystem::Status BaseSubsystem::getStatus() const {
    rwLock.RLock();
    const auto rc = status;
    rwLock.RUnlock();
    return rc;
}

const char * const BaseSubsystem::statusString() const {
    const auto status = getStatus();
    switch(status) {
        case INIT:
            return "INIT";
            break;
        case READY:
            return "READY";
            break;
        case FAULT:
            return "FAULT";
            break;
        case RUNNING:
            return "RUNNING";
            break;
        case STOPPED:
            return "STOPPED";
            break;
        default:
            return "INVALID STATUS";
            break;
    }
}


void BaseSubsystem::setStatus(BaseSubsystem::Status newStatus) {
    rwLock.Lock();
    status = newStatus;
    rwLock.UnLock();
}

bool BaseSubsystem::lowPowerMode() {
    return false;
}

BaseSubsystem::Status BaseSubsystem::stop() {
    setStatus(STOPPED);
    return getStatus();
}

TickableSubsystem::~TickableSubsystem() {}

// Not meaningful in tickable subsystem
BaseSubsystem::Status TickableSubsystem::start() {
    if (getStatus() != FAULT) {
        setStatus(RUNNING);
    }
    return getStatus();
}

ThreadedSubsystem::ThreadedSubsystem() : taskHandle(0) {
}

ThreadedSubsystem::~ThreadedSubsystem() {}

BaseSubsystem::Status ThreadedSubsystem::start() {
    auto taskFn = [](void* s) -> void {
        // avoid every thread starting at once
        constexpr auto minimum_number = 1;
        constexpr auto maximum_number = (100L * configTICK_RATE_HZ) / 1000L;
        const auto delay = esp_random() % (maximum_number + 1 - minimum_number) + minimum_number;
        vTaskDelay(delay);

        auto self = static_cast<ThreadedSubsystem*>(s);
        auto param = self->taskParameter();
        self->taskFunction(param);
    };
    switch(getStatus()) {
        case READY:
            taskHandle = xTaskCreateStaticPinnedToCore(
                taskFn,
                name,
                STACK_SIZE,
                this,
                taskPriority(),
                taskStack,
                &taskBuffer,
                core());
        if (taskHandle == nullptr) {
            setStatus(FAULT);
        } else {
            setStatus(RUNNING);
        }
        break;

        case STOPPED:
        vTaskResume(taskHandle);
        setStatus(RUNNING);
        break;
    }
    return getStatus();
}

BaseSubsystem::Status ThreadedSubsystem::stop() {
    vTaskSuspend(taskHandle);
    return BaseSubsystem::stop();
}

int ThreadedSubsystem::taskPriority() const {
    return tskIDLE_PRIORITY;
}

void * const ThreadedSubsystem::taskParameter() {
    return nullptr;
}

/**
 * @brief default implementation alternates cores
 *
 * @return int the core to run on
 */
int ThreadedSubsystem::core() {
    static int alternator = 0;
    auto core = alternator;
    alternator += alternator % 2;
    return core;
}

SubsystemManagerClass::SubsystemManagerClass() {}
SubsystemManagerClass::~SubsystemManagerClass() {}

SubsystemManagerClass::Spec::Spec(BaseSubsystem *subsys, BaseSubsystem** deps) : subsystem(subsys), deps(deps), next(NULL) {}


void SubsystemManagerClass::addSubsystem(Spec *spec) {
    // constructor calling order is undefined and subsystemanagerclass may not be constructed already
    static bool setup = false;
    if (setup == false) {
        specs = NULL;
        setup = true;
    }
    if (spec == NULL || spec->subsystem == NULL) {
        return; // error case, but no abilty to gripe
    }
    if (specs != NULL) { // non empty list, prepend
        spec->next = specs;
    } else {
        spec->next = NULL;
    }
    specs = spec;
}

BaseSubsystem::Status SubsystemManagerClass::setup() {
    auto spec = specs;

    #ifdef MANAGER_DEBUG
    Serial.println("in setup, specs dump:");
    while (spec != NULL && spec->subsystem != NULL) {
        Serial.printf("'%s' depends on (", spec->subsystem->name);
        for (auto i = 0; spec && spec->deps && spec->deps[i]; i++) {
            auto s = spec->deps[i];
            if (s && s->name) {
                Serial.printf("'%s', ", s->name);
            }
        }
        Serial.println(")");
        spec = spec->next;
    }
    spec = specs;
    Serial.println();
    #endif

    while (spec != NULL && spec->subsystem != NULL) {
        descendAndStartOrSetup(spec, READY);
        spec = spec->next;
    }
    setStatus(READY);
    return getStatus();
}

BaseSubsystem::Status SubsystemManagerClass::start() {
    auto spec = specs;
    while (spec != NULL && spec->subsystem != NULL) {
        descendAndStartOrSetup(spec, RUNNING);
        spec = spec->next;
    }
    setStatus(RUNNING);
    return getStatus();
}

void SubsystemManagerClass::iterateSubsystems(SubsystemFn fn, void *args) {
    rwLock.RLock();
    for (Spec *spec = specs; spec; spec=spec->next) {
        fn(spec->subsystem, args);
    }
    rwLock.RUnlock();
}


SubsystemManagerClass::Spec* SubsystemManagerClass::findSpecBySubsystem(BaseSubsystem *needle) {
    auto spec = specs;
    while (spec) {
        if (spec->subsystem == needle) {
            return spec;
        }
        spec = spec->next;
    }
    // p is null -- we didn't find it
    return NULL;
}

void SubsystemManagerClass::descendAndStartOrSetup(Spec *spec, BaseSubsystem::Status desiredState, int depth) {
    if (specs == NULL || spec == NULL || spec->subsystem == NULL) {
        return;
    }
    const auto subsystem = spec->subsystem;
    const auto name = subsystem->name;

    auto deps = spec->deps;
    for (auto i = 0; deps && deps[i] && depth < 8; i++) {
        const auto s = findSpecBySubsystem(deps[i]);
        descendAndStartOrSetup(s, desiredState, depth++);
    }
    const auto status = subsystem->getStatus();
    if ((status == desiredState) || (status == FAULT || status == STOPPED))  {
        return;
    }
    if (desiredState == READY && status == INIT) {
        #ifdef MANAGER_DEBUG
        Serial.printf("setup subsystem '%s' (status: %d) -> ", subsystem->name, status);
        auto newstatus = subsystem->setup();
        Serial.printf(" (status: %d)\n", newstatus);
        #else
        subsystem->setup();
        #endif
    }
    if (desiredState == RUNNING && status == READY) {
        #ifdef MANAGER_DEBUG
        Serial.printf("start subsystem '%s' (status: %d) -> ", subsystem->name, status);
        auto newstatus = subsystem->start();
        Serial.printf(" (status: %d)\n", newstatus);
        #else
        subsystem->start();
        #endif
    }
}



SubsystemManagerClass SubsystemManager;
