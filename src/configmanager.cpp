#include "configmanager.h"
#include "esp_system.h"
#include "log.h"

ConfigManagerClass ConfigManager;

// for json
static constexpr char NAME_STR[] =              "name";
static constexpr char ID_STR[] =                "ID";
static constexpr char DELAY_SECONDS_STR[] =     "delaySeconds";
static constexpr char MAIN_DEPLOY_ALT_STR[] =   "mainDeployAlt";
static constexpr char AIR_START_DELAY_STR[] =   "airStartDelay";
static constexpr char BEEPER_FREQUENCY_STR[] =  "beeperFrequency";
static constexpr char ESSID_STR[] =             "ESSID";
static constexpr char PASSWORD_STR[] =          "password";
static constexpr char APMODE_STR[] =            "APMode";

static constexpr char WIFI_CONFIG_STR[] =       "WIFIConfig";
static constexpr char PYRO_CONFIG_STR[] =       "PyroConfig";

// default values
static constexpr int defaultBeeperFrequency =   2500;
static constexpr char defaultPassword[] =       "12345678";
static constexpr bool defaultAPMode =           true;

// preferences key
static constexpr char ConfigKey[] =             "config-data";

bool operator== (const ConfigData::WIFIConfig &a, const ConfigData::WIFIConfig &b){
    if (strncmp(a.ESSID, b.ESSID, sizeof(a.ESSID)) ||
        strncmp(a.password, b.password, sizeof(a.password)) ||
        a.APMode != b.APMode) {
        return false;
    }
    return true;
}

bool canConvertFromJson(JsonVariantConst src, const ConfigData::WIFIConfig&) {
    return src[ESSID_STR].is<const char*>() && 
    src[PASSWORD_STR].is<const char*>() &&
    src[APMODE_STR].is<bool>();
}

ConfigData::WIFIConfig::WIFIConfig(const ConfigData::WIFIConfig &other) {
    strncpy(ESSID, other.ESSID, sizeof(ESSID));
    strncpy(password, other.password, sizeof(password));
    APMode = other.APMode;
}

ConfigData::ConfigData() : beeperFrequency(defaultBeeperFrequency) {
    // find the ID
    uint8_t baseMac[6];
	// Get MAC address for WiFi station
	esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
	snprintf(ID, sizeof(ID), "%02X:%02X:%02X:%02X:%02X:%02X",
    baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    strncpy(name, ID, sizeof(name));

    // default WIFIConfig
    wifiConfig.APMode = defaultAPMode;
    strncpy(wifiConfig.password, defaultPassword, sizeof(wifiConfig.password));
    snprintf(wifiConfig.ESSID, sizeof(wifiConfig.ESSID), "ldrc-%02X:%02X:%02X:%02X:%02X:%02X", baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
}

bool convertToJson(const ConfigData::WIFIConfig &src, JsonVariant dst) {
    dst[ESSID_STR].set(src.ESSID);
    dst[PASSWORD_STR].set(src.password);
    dst[APMODE_STR] = src.APMode;
    return true;
}

void convertFromJson(JsonVariantConst src, ConfigData::WIFIConfig &dst) {
    if (src[ESSID_STR].is<const char*>()) {
        auto essid = src[ESSID_STR].as<const char*>();
        strncpy(dst.ESSID, essid, sizeof(dst.ESSID));
    }
    if (src[PASSWORD_STR].is<const char*>()) {
        auto password = src[PASSWORD_STR].as<const char*>();
        strncpy(dst.password, password, sizeof(dst.password));
    }
    if (src[APMODE_STR].is<bool>()) {
        auto apmode = src[APMODE_STR].as<bool>();
        dst.APMode = apmode;
    }
}


bool convertToJson(const ConfigData &src, JsonVariant dst) {
    dst[NAME_STR] = src.name;
    dst[ID_STR] = src.ID;
    dst[BEEPER_FREQUENCY_STR] = src.beeperFrequency;
    dst[WIFI_CONFIG_STR] = src.wifiConfig;

    auto arr = dst[PYRO_CONFIG_STR].to<JsonArray>();
    for (auto i = 0; i < PyroChannelConfig::maxPyroChannels; i++) {
        arr.add(src.pyroConfigs[i]);
    }

    return true;
}

void convertFromJson(JsonVariantConst src, ConfigData &dst) {
    if (src[NAME_STR].is<const char*>()) {
        auto name = src[NAME_STR].as<const char*>();
        strncpy(dst.name, name, sizeof(dst.name));
    }
    if (src[BEEPER_FREQUENCY_STR].is<int>()) {
        auto beeperFrequency = src[BEEPER_FREQUENCY_STR].as<int>();
        dst.beeperFrequency = beeperFrequency;
    }
    if (src[WIFI_CONFIG_STR].is<ConfigData::WIFIConfig>()) {
        dst.wifiConfig = src[WIFI_CONFIG_STR].as<ConfigData::WIFIConfig>();
    }
    if (src[PYRO_CONFIG_STR].is<JsonArray>()) {
        for (auto i = 0; i < PyroChannelConfig::maxPyroChannels; i++) {
            dst.pyroConfigs[i] = src[PYRO_CONFIG_STR][i];
        }
    }
}

ConfigData::ConfigData(const ConfigData& other) {
    strncpy(name, other.name, sizeof(name));
    strncpy(ID, other.ID, sizeof(ID));
    beeperFrequency = other.beeperFrequency;
    wifiConfig = other.wifiConfig;
    for (auto i = 0; i < PyroChannelConfig::maxPyroChannels; i++) {
        pyroConfigs[i] = other.pyroConfigs[i];
    }
}

ConfigManagerClass::ConfigManagerClass() : BaseSubsystem(), DataProvider<ConfigData>(rwLock) {
    name = "configmgr";
    static BaseSubsystem* deps[] = {&LogWriter, NULL};
    static SubsystemManagerClass::Spec spec(this, deps);
    SubsystemManager.addSubsystem(&spec);
}

ConfigManagerClass::~ConfigManagerClass() {
}

BaseSubsystem::Status ConfigManagerClass::setup() {
    preferences.begin("ldrc_configmgr", false);
    setStatus(READY);
    return getStatus();
}

BaseSubsystem::Status ConfigManagerClass::start() {
    load();
    setStatus(RUNNING);
    return getStatus();
}

void ConfigManagerClass::save() {
    Log.noticeln("configmgr::save");

    rwLock.RLock();
    preferences.putBytes(ConfigKey, &data, sizeof(data));
    rwLock.RUnlock();
}

void ConfigManagerClass::load() {
    Log.noticeln("configmgr::load");

    rwLock.Lock();
    auto rc = preferences.getBytes(ConfigKey, &data, sizeof(data));
    rwLock.UnLock();
    if (rc == 0) {
        // loading error use default
        data = ConfigData();
    }
    callCallbacks();
}

void ConfigManagerClass::setConfigData(const ConfigData &other) {
    rwLock.Lock();
    data = other;
    rwLock.UnLock();
    callCallbacks();
}

void ConfigManagerClass::onUpdate() {
    // FIXME: disabled for now
    //save();
}