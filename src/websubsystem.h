#pragma once

#include "subsystem.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

class WebSubsystemClass : public ThreadedSubsystem {
    public:
        WebSubsystemClass();
        virtual ~WebSubsystemClass();

        BaseSubsystem::Status setup();
        BaseSubsystem::Status start();

        void stop();
        void cleanup();

    protected:
        void taskFunction(void *arg);

    private:
        AsyncWebServer server;
        AsyncWebSocket ws;
};

extern WebSubsystemClass WebSubsystem;