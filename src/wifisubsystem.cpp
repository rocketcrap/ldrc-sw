#include "wifisubsystem.h"
#include "configmanager.h"
#include "eventmanager.h"
#include "websubsystem.h"
#include "log.h"
#include "WiFi.h"

WifiSubsystemClass WifiSubsystem;

WifiSubsystemClass::WifiSubsystemClass() {
    name = "wifi";
    static BaseSubsystem* deps[] = {&LogWriter, &ConfigManager, /* &WebSubsystem, */ NULL}; //FIXME
    static SubsystemManagerClass::Spec spec(this, deps);
    SubsystemManager.addSubsystem(&spec);
}

WifiSubsystemClass::~WifiSubsystemClass() {
}

BaseSubsystem::Status WifiSubsystemClass::start() {
    Log.noticeln("wifi running");
    setStatus(RUNNING);
    return getStatus();
}

BaseSubsystem::Status WifiSubsystemClass::stop() {
    if (wifiConfig.APMode) {
        WiFi.softAPdisconnect(true);
    } else {
        WiFi.disconnect(true);
    }
    setStatus(STOPPED);
    return getStatus();
}

BaseSubsystem::Status WifiSubsystemClass::setup() {
    setStatus(READY);

    // the conclusion was basically that wifi changes should just be a reboot

    ConfigManager.registerCallback([](const ConfigData& config, void *arg) {
        Log.noticeln("wifisubystem configmgr callback");
        auto self = static_cast<WifiSubsystemClass*>(arg);

        self->rwLock.Lock();

        auto changed = true;
        if (config.wifiConfig == const_cast<const WifiSubsystemClass*>(self)->wifiConfig) {
            changed = false;
        }
        if (changed) {
            self->wifiConfig = config.wifiConfig;
        }

        self->rwLock.UnLock();

        if (!changed) {
            return;
        }

        // you can't delay here to get post to complete, because the calling thread is webubsystem
        WebSubsystem.stop();

        const auto essid = self->wifiConfig.ESSID;
        const auto password = self->wifiConfig.password;
        if (self->wifiConfig.APMode) {
            WiFi.mode(WIFI_AP);
            Log.noticeln("Starting AP\tESSID:'%s'\tPassword:'%s'", essid, password);
            WiFi.softAP(essid, password);
            auto IP = WiFi.softAPIP();
            Log.noticeln("Server IP: %s", IP.toString().c_str());
        } else {
            Log.noticeln("Starting Client");
            WiFi.mode(WIFI_STA);
        }
        WiFi.begin(essid, password);
    }, this);

    WiFi.onEvent([this](arduino_event_id_t event, arduino_event_info_t info) {
        Log.noticeln("Client IP: %s", WiFi.localIP().toString());
    }, ARDUINO_EVENT_WIFI_STA_GOT_IP);

    WiFi.onEvent([this](arduino_event_id_t event, arduino_event_info_t info) {
        WebSubsystem.start();
    }, ARDUINO_EVENT_WIFI_STA_CONNECTED);

    WiFi.onEvent([this](arduino_event_id_t event, arduino_event_info_t info) {
        WebSubsystem.start();
    }, ARDUINO_EVENT_WIFI_AP_START);

    EventManager.subscribe([](const Event& ev, void *arg) {
        auto self = static_cast<WifiSubsystemClass*>(arg);

        WebSubsystem.stop();
        self->stop();
    }, Event::EventType::LIFTOFF_EVENT, this);

    return getStatus();
}
