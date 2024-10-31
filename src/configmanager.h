#pragma once

#include <Arduino.h>
#include "subsystem.h"
#include "rwlock.h"
#include "ArduinoJson.h"
#include "Preferences.h"
#include "pyrochannelconfig.h"


struct __attribute__((packed)) ConfigData {
    ConfigData();
    ConfigData(const ConfigData& other);
    const char* modeToString() const;
    void modeFromString(const char* str);

    char name[32+1];
    char ID[18];
    int beeperFrequency;

    struct __attribute__((packed)) WIFIConfig {
        WIFIConfig(): ESSID("UNSET"), password("UNSETUNSET") {}
        WIFIConfig(const WIFIConfig &other);
        char ESSID[32+1];
        char password[64];
        bool APMode;

        friend bool operator== (const WIFIConfig &a, const WIFIConfig &b);
    } wifiConfig;

    PyroChannelConfig pyroConfigs[PyroChannelConfig::maxPyroChannels];
};

bool canConvertFromJson(JsonVariantConst src, const ConfigData::WIFIConfig&);
bool convertToJson(const ConfigData::WIFIConfig &src, JsonVariant dst);
void convertFromJson(JsonVariantConst src, ConfigData::WIFIConfig &dst);

bool convertToJson(const ConfigData &src, JsonVariant dst);
void convertFromJson(JsonVariantConst src, ConfigData &dst);

class ConfigManagerClass : public BaseSubsystem, public DataThing<ConfigData> {
    public:
        ConfigManagerClass();
        virtual ~ConfigManagerClass();
        ConfigManagerClass& operator=(const ConfigManagerClass& other) = delete;

        BaseSubsystem::Status setup();
        BaseSubsystem::Status start();

        void setConfigData(const ConfigData &other);

    protected:
        virtual void onUpdate();

    private:
        Preferences preferences;

        void save();
        void load();
};

extern ConfigManagerClass ConfigManager;
