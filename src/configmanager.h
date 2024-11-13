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

    static constexpr size_t NAME_LEN = 32+1;
    static constexpr size_t ID_LEN = 18;

    char name[NAME_LEN];
    char ID[ID_LEN];
    int beeperFrequency;

    struct __attribute__((packed)) WIFIConfig {
        static constexpr size_t ESSID_LEN = 32+1;
        static constexpr size_t PASSWORD_LEN = 64;

        WIFIConfig(): ESSID("UNSET"), password("UNSETUNSET") {}
        WIFIConfig(const WIFIConfig &other);
        char ESSID[ESSID_LEN];
        char password[PASSWORD_LEN];
        bool APMode;

        friend bool operator== (const WIFIConfig &a, const WIFIConfig &b);
    } wifiConfig;

    struct __attribute__((packed)) OwnerInformation {
        static constexpr size_t NAME_LEN = 16;
        static constexpr size_t EMAIL_LEN = 16;
        char name[NAME_LEN];
        char email[EMAIL_LEN];
    } ownerInformation;

    PyroChannelConfig pyroConfigs[PyroChannelConfig::maxPyroChannels];
};

bool canConvertFromJson(JsonVariantConst src, const ConfigData::WIFIConfig&);
bool convertToJson(const ConfigData::WIFIConfig &src, JsonVariant dst);
void convertFromJson(JsonVariantConst src, ConfigData::WIFIConfig &dst);

bool convertToJson(const ConfigData &src, JsonVariant dst);
void convertFromJson(JsonVariantConst src, ConfigData &dst);

class ConfigManagerClass : public BaseSubsystem, public DataProvider<ConfigData> {
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
