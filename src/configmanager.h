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

    enum Mode {
        FLIGHT_COMPUTER,
        GROUND_STATION,
        RELAY,
    } mode;

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

const char* modeToString(const ConfigData::Mode mode);
bool convertToJson(const ConfigData::Mode &src, JsonVariant dst);
void convertFromJson(JsonVariantConst src, ConfigData::Mode &dst);

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

        /**
         * @brief convenience function to get the Mode
         *
         * @return ConfigData::Mode mode of device
         */
        ConfigData::Mode getMode() const;

        void setConfigData(const ConfigData &other);

    protected:
        virtual void onUpdate();

    private:
        Preferences preferences;

        void save();
        void load();
};

extern ConfigManagerClass ConfigManager;
