#include "websubsystem.h"
#include "gps-subsystem.h"
#include "configmanager.h"
#include "wifisubsystem.h"
#include "statusmanager.h"
#include "log.h"
//#include "radio.h"
//#include "fileLogging.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>

#include <LittleFS.h>

struct SpiRamAllocator : ArduinoJson::Allocator {
    void* allocate(size_t size) override {
        return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    }

    void deallocate(void* pointer) override {
        heap_caps_free(pointer);
    }

    void* reallocate(void* ptr, size_t new_size) override {
        return heap_caps_realloc(ptr, new_size, MALLOC_CAP_SPIRAM);
    }
};
static SpiRamAllocator allocator;

WebSubsystemClass WebSubsystem;

static inline void notFound(AsyncWebServerRequest *request)  {
    request->send(404, "text/plain", "Not found");
}

static inline AsyncResponseStream *beginJSON(AsyncWebServerRequest *request) {
    return request->beginResponseStream("application/json");
}

WebSubsystemClass::WebSubsystemClass() : server(80), ws("/ws") {
    name = "web";
    static BaseSubsystem* deps[] = {&LogWriter, &WifiSubsystem, &ConfigManager, &StatusManager, NULL};
    static SubsystemManagerClass::Spec spec(this, deps);
    SubsystemManager.addSubsystem(&spec);
}

WebSubsystemClass::~WebSubsystemClass() {
}

BaseSubsystem::Status WebSubsystemClass::setup() {
    server.on("/gps", HTTP_GET, [](AsyncWebServerRequest *request) {
        static JsonDocument json(&allocator);
        auto response = beginJSON(request);
        auto fix = GPSSubsystem.getFix();

        json.clear();
        json.set(fix);
        serializeJsonPretty(json, *response);
        request->send(response);
     });

    server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        static JsonDocument json(&allocator);
        auto response = beginJSON(request);
        ConfigManager.readData([](const ConfigData& data, void *arg) {
            auto j = static_cast<JsonDocument*>(arg);
            j->set(data);
        }, &json);
        serializeJsonPretty(json, *response);
        request->send(response);
    });

    static auto jsonWebHandler = AsyncCallbackJsonWebHandler("/config", [](AsyncWebServerRequest *request, JsonVariant &json) {
        // FIXME: never returns if you change wifi stuff
        Log.noticeln("config json post");
        auto response = beginJSON(request);
        // here we set the config manager, which calls the hooks prior to us sending a response
        // FIXME: this is all stupid -- network changes should just require a reboot except for launch->turn off wifi
        // but it works

        auto newConfig = json.as<ConfigData>();

        // send it along b/c if we set it, we'll never get the response
        serializeJsonPretty(json, *response);

        ConfigManager.setConfigData(newConfig);
        return;

        // this can't work because if we switch the wifi config, we won't send the response
        // ConfigManager.readData([](const ConfigData& data, void *arg){
        //     auto j = static_cast<JsonVariant*>(arg);
        //     j->set(data);
        // }, &json);
        // serializeJsonPretty(json, *response);
    });
    AsyncCallbackJsonWebHandler *configPostHandler = &jsonWebHandler;
    server.addHandler(configPostHandler);

    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        static JsonDocument json(&allocator);

        json.clear();
        auto response = beginJSON(request);
        StatusManager.readData([](const StatusPacket &pkt, void *arg) {
            auto j = static_cast<JsonDocument*>(arg);
            j->set(pkt);
        }, &json);
        serializeJsonPretty(json, *response);
        request->send(response);
    });

    server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request) {
        Log.noticeln("rebooting on request");
        ESP.restart();
    });


    // // TODO: maybe we should use server side events instead
    // // see https://randomnerdtutorials.com/esp32-web-server-sent-events-sse/

    // Radio.registerCallback([](const ReceivePacket &rcvpkt, void* arg) {
    //     constexpr size_t maxJson = 512; // recommended size for status, plus padding for relay
    //     static StaticJsonDocument<maxJson> json;
    //     static uint8_t buffer[maxJson];

    //     auto self = static_cast<WebSubsystemClass*>(arg);
    //     if (self->getStatus() != BaseSubsystem::RUNNING) {
    //         return;
    //     }
    //     json.clear();
    //     json["packet"] = rcvpkt;

    //     auto len = serializeJsonPretty(json, buffer, sizeof(buffer));
    //     self->ws.textAll(buffer, len);
    // }, this);

    GPSSubsystem.registerCallback([](const GPSFix &fix, void *arg) {
        constexpr size_t maxJson = 200;
        static JsonDocument json(&allocator);
        static uint8_t buffer[maxJson];
        static auto last = millis();

        auto self = static_cast<WebSubsystemClass*>(arg);
        auto now = millis();
        if ((now - last) < 1000) { // max rate is once per second
            return;
        }
        if (self->getStatus() != BaseSubsystem::RUNNING) {
            return;
        }
        last = now;
        json.clear();
        json["gps"] = fix;

        auto len = serializeJsonPretty(json, buffer, sizeof(buffer));
        self->ws.textAll(buffer, len);
    }, this);
    server.addHandler(&ws);

    //auto filename = getFilename();
    auto fs = &LittleFS;
    if (fs) {
        // if (filename) {
        //     server.serveStatic("/logfile", *fs, filename);
        // }
        server.serveStatic("/", *fs, "/www/").setDefaultFile("index.html");
    }
    server.onNotFound(notFound);

    setStatus(STOPPED); //otherwise it will be started by subsystem manager
    return getStatus();
}

BaseSubsystem::Status WebSubsystemClass::start() {
    server.begin();
    setStatus(RUNNING);
    return getStatus();
}

void WebSubsystemClass::stop() {
    server.end();
    setStatus(BaseSubsystem::STOPPED);
}

void WebSubsystemClass::cleanup() {
    ws.cleanupClients();
}

void WebSubsystemClass::taskFunction(void *arg) {
    while(1) {
        delay(250);  //4 times a second seems often enough
        cleanup();
    }
}