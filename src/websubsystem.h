#pragma once

#include "subsystem.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

//FIXME: convert to tickable

class WebSubsystemClass : public TickableSubsystem {
    public:
        WebSubsystemClass();
        virtual ~WebSubsystemClass();

        BaseSubsystem::Status setup();
        BaseSubsystem::Status start();
        BaseSubsystem::Status stop();
        BaseSubsystem::Status tick();

    protected:
        void cleanup();

    private:
        AsyncWebServer server;
        AsyncWebSocket ws;
};

extern WebSubsystemClass WebSubsystem;