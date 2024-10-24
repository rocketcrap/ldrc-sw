#include "pyrosubsystem.h"
#include "eventmanager.h"
#include "statusmanager.h"
#include "configmanager.h"
#include "statemanager.h"
#include "log.h"
#include "pins.h"

PyroManagerClass PyroManager;

// FIXME: pindefinitions
static PyroManagerClass::PyroChannel ChannelOne(QCHAN1, VCHAN1);
static PyroManagerClass::PyroChannel ChannelTwo(QCHAN2, VCHAN2);
//static PyroManagerClass::PyroChannel ChannelThree(1, 2);

static constexpr char CHANNEL_TYPE_STR[] =      "channelType";
static constexpr char DELAY_SECONDS_STR[] =     "delayS";
static constexpr char MAIN_ALT_STR[] =          "mainAlt";
static constexpr char BURNOUT_NUM_STR[] =       "burnoutNum";
static constexpr char AIRSTART_ALT_STR[]  =     "airstartAlt";
static constexpr char AIRSTART_ANGL_STR[] =     "airstartAngle";
static constexpr char AIRSTART_VEL_STR[] =      "airstartVel";

static constexpr char DISABLED_CHAN_STR[] =     "disabled";
static constexpr char DROGUE_CHAN_STR[] =       "drogue";
static constexpr char MAIN_CHAN_STR[] =         "main";
static constexpr char AIRSTART_CHAN_STR[] =     "airstart";
static constexpr char BURNOUT_CHAN_STR[] =      "burnout";

static const char* chanTypeToString(PyroChannelConfig::ChannelType chan) {
    switch (chan) {
        case PyroChannelConfig::DISABLED_CHANNEL:
            return DISABLED_CHAN_STR;
        break;

        case PyroChannelConfig::DROUGE_CHANNEL:
            return DROGUE_CHAN_STR;
        break;

        case PyroChannelConfig::MAIN_CHANNEL:
            return MAIN_CHAN_STR;
        break;

        case PyroChannelConfig::AIRSTART_CHANNEL:
            return AIRSTART_CHAN_STR;
        break;

        case PyroChannelConfig::BURNOUT_CHANNEL:
            return BURNOUT_CHAN_STR;
        break;

        default:
            return "invalid";
        break;
    }
}

static PyroChannelConfig::ChannelType stringToChanType(const char *str) {
    if (!strcmp(DISABLED_CHAN_STR, str)) {
        return PyroChannelConfig::DISABLED_CHANNEL;
    }
    if (!strcmp(DROGUE_CHAN_STR, str)) {
        return PyroChannelConfig::DROUGE_CHANNEL;
    }
    if (!strcmp(MAIN_CHAN_STR, str)) {
        return PyroChannelConfig::MAIN_CHANNEL;
    }
    if (!strcmp(AIRSTART_CHAN_STR, str)) {
        return PyroChannelConfig::AIRSTART_CHANNEL;
    }
    if (!strcmp(BURNOUT_CHAN_STR, str)) {
        return PyroChannelConfig::BURNOUT_CHANNEL;
    }
    return PyroChannelConfig::DISABLED_CHANNEL;
}

bool convertToJson(const PyroChannelConfig &src, JsonVariant dst) {
    dst[CHANNEL_TYPE_STR] = chanTypeToString(src.channelType);
    dst[DELAY_SECONDS_STR] = src.delaySeconds;
    dst[MAIN_ALT_STR] = src.mainAlt;
    dst[BURNOUT_NUM_STR] = src.burnoutNumber;
    dst[AIRSTART_ALT_STR] = src.airStartLockoutAltitude;
    dst[AIRSTART_ANGL_STR] = src.airStartLockoutAngle;
    dst[AIRSTART_VEL_STR] = src.airStartLockoutVelocity;
    return true;
}

void convertFromJson(JsonVariantConst src, PyroChannelConfig &dst) {
    if (src[CHANNEL_TYPE_STR].is<const char*>()) {
        dst.channelType = stringToChanType(src[CHANNEL_TYPE_STR]);
    }
    if (src[DELAY_SECONDS_STR].is<int>()) {
        dst.delaySeconds = src[DELAY_SECONDS_STR].as<decltype(dst.delaySeconds)>();
    }
    if (src[MAIN_ALT_STR].is<int>()) {
        dst.mainAlt = src[MAIN_ALT_STR].as<decltype(dst.mainAlt)>();
    }
    if (src[BURNOUT_NUM_STR].is<int>()) {
        dst.burnoutNumber = src[BURNOUT_NUM_STR].as<decltype(dst.burnoutNumber)>();
    }
    if (src[AIRSTART_ALT_STR].is<int>()) {
        dst.airStartLockoutAltitude = src[AIRSTART_ALT_STR].as<decltype(dst.airStartLockoutAltitude)>();
    }
    if (src[AIRSTART_ANGL_STR].is<int>()) {
        dst.airStartLockoutAngle = src[AIRSTART_ANGL_STR].as<decltype(dst.airStartLockoutAngle)>();
    }
    if (src[AIRSTART_VEL_STR].is<int>()) {
        dst.airStartLockoutVelocity = src[AIRSTART_VEL_STR].as<decltype(dst.airStartLockoutVelocity)>();
    }
}

PyroManagerClass::PyroManagerClass() : pyroChannels{&ChannelOne, &ChannelTwo, /* &ChannelThree, */ NULL},
    numPyroChannels(0), liftOffDetected(false), apogeeDetected(false), pyroArmed(false),
    indicators(numLED, LED_DATA, LED_CLK, DOTSTAR_BGR) {
    name = "pyroManager";
    static BaseSubsystem* deps[] = {&EventManager, &StatusManager, &ConfigManager, &LogWriter, NULL};
    static SubsystemManagerClass::Spec spec(this, deps);
    SubsystemManager.addSubsystem(&spec);

    // initialize the number of pyrochannels
    for (auto i = 0; i < maxPyroChannels; i++) {
        if (pyroChannels[i] != NULL) {
            pyroChannels[i]->chanNum = i;
            numPyroChannels++;
        }
    }
}

PyroManagerClass::~PyroManagerClass() {
}

BaseSubsystem::Status PyroManagerClass::setup() {
    // TODO: consider if pyromanager should care about lowpower
    auto eventMask = Event::ARM_EVENT | Event::DISARM_EVENT | Event::LIFTOFF_EVENT | Event::BURNOUT_EVENT | Event::BURNOUT_EVENT | Event::LANDING_EVENT;

    rwLock.Lock();

    EventManager.subscribe([](const Event &event, void *ctx){
        auto self = static_cast<PyroManagerClass*>(ctx);
        switch (event.eventType)
        {
            case Event::ARM_EVENT:
                self->onArmed();
                break;

            case Event::DISARM_EVENT:
                self->onDisarmed();
                break;

            case Event::LIFTOFF_EVENT:
                self->onLiftOff();
                break;

            case Event::BURNOUT_EVENT:
                self->onBurnout(event);
                break;

            case Event::APOGEE_EVENT:
                self->onApogee();
                break;

            case Event::LANDING_EVENT:
                self->onLanding();
                break;

            default:
                break;
        }
    }, eventMask, this);

    Serial.println("postSubscribe");

    ConfigManager.readData([](const ConfigData &config, void *p) {
        auto self = static_cast<PyroManagerClass*>(p);

        self->forEachChanNoLock([](PyroChannel *chan, size_t i, PyroManagerClass *self, void *p) {
            auto config = static_cast<const ConfigData *>(p);
            self->pyroChannels[i]->config = config->pyroConfigs[i];
        }, const_cast<ConfigData*>(&config));
    }, this);


    rwLock.UnLock();

    ConfigManager.registerCallback([](const ConfigData &config, void *p) {
        auto self = static_cast<PyroManagerClass*>(p);

        self->onDisarmed(); // changing config disarms system
        self->forEachChanLock([](PyroChannel *chan, size_t i, PyroManagerClass *self, void *p) {
            auto config = static_cast<const ConfigData *>(p);
            self->pyroChannels[i]->config = config->pyroConfigs[i];
        }, const_cast<ConfigData*>(&config));
    }, this);

    indicators.begin();
    indicators.show();

    setStatus(READY);
    return getStatus();
}

BaseSubsystem::Status PyroManagerClass::tick() {
    // check if continuity has changed on channels
    tickContinuityChanges();

    // check if a firing delay is set on channels and fire if expired
    tickFiringDelay();

    // check if firing channels duration is up
    tickFiringDuration();

    // check if we have reached target altitude
    tickAltitudeTrigger();

    return getStatus();
}

bool PyroManagerClass::allConfiguredChannelsContinuity() {
    bool rc = true;
    forEachChanLock([](PyroManagerClass::PyroChannel *chan, size_t i, PyroManagerClass *self, void *ctx) {
        auto rc = static_cast<bool*>(ctx);
        if (chan->config.channelType != PyroChannelConfig::DISABLED_CHANNEL) {
            if (!chan->checkContinuity()) {
                *rc = false;
            }
        }
    }, &rc);
    return rc;
}

void PyroManagerClass::tickFiringDuration() {
    if (armed() && liftedOff()) {
        forEachChanLock([](PyroChannel* chan, size_t i, PyroManagerClass *self, void *ctx) {
            if (chan->firingDurationExpired()) {
                chan->stopFiring();
            }
        }, NULL);
    }
}

void PyroManagerClass::tickFiringDelay() {
    if (armed() && liftedOff()) {
        forEachChanLock([](PyroChannel* chan, size_t i, PyroManagerClass *self, void *ctx) {
            if (chan->firingDelayExpired()) {
                chan->startFiring();
            }
        }, NULL);
    }
}

void PyroManagerClass::tickContinuityChanges() {
    forEachChanLock([](PyroChannel* chan, size_t i, PyroManagerClass *self, void *ctx) {
        chan->checkContinuity();
    }, NULL);
}

void PyroManagerClass::tickAltitudeTrigger() {
    if (armed() && liftedOff() && postApogee()) {
        float alt = StateManager.getAGL();
        forEachChanLock([](PyroChannel* chan, size_t i, PyroManagerClass *self, void *ctx) {
            auto altitude = *(static_cast<float*>(ctx));
            if (chan->config.channelType == PyroChannelConfig::MAIN_CHANNEL) {
                if (!chan->firing && (altitude <= chan->config.mainAlt)) {
                    chan->startFiringDelay();
                }
            }
        }, &alt);
    }
}

bool PyroManagerClass::armed() const {
    rwLock.RLock();
    auto ret = pyroArmed;
    rwLock.RUnlock();
    return ret;
}

void PyroManagerClass::onArmed() {
    rwLock.Lock();
    pyroArmed = true;
    indicators.clear();
    indicators.show();
    rwLock.UnLock();
}

void PyroManagerClass::onDisarmed() {
    rwLock.Lock();

    forEachChanNoLock([](PyroChannel *chan, size_t i, PyroManagerClass *self, void *p) {
        self->pyroChannels[i]->stopFiring();
    }, NULL);

    pyroArmed = false;

    rwLock.UnLock();
}

bool PyroManagerClass::liftedOff() const {
    rwLock.RLock();
    auto ret = liftOffDetected;
    rwLock.RUnlock();
    return ret;
}

bool PyroManagerClass::postApogee() const {
    rwLock.RLock();
    auto ret = apogeeDetected;
    rwLock.RUnlock();
    return ret;
}

void PyroManagerClass::onLiftOff() {
    rwLock.Lock();
    liftOffDetected = true;
    rwLock.UnLock();
}

void PyroManagerClass::onBurnout(const Event &event) {
    if (liftedOff() == false || armed() == false) {
        return;
    }
    forEachChanLock([](PyroChannel* chan, size_t i, PyroManagerClass *self, void *ctx) {
        auto num = *(static_cast<int32_t*>(ctx));
        switch (chan->config.channelType) {
            case PyroChannelConfig::BURNOUT_CHANNEL:
                if (chan->config.burnoutNumber == num) {
                    chan->startFiringDelay();
                }
            break;

            case PyroChannelConfig::AIRSTART_CHANNEL:
                if (chan->config.burnoutNumber != num) {
                    Log.noticeln("not the right burnout");
                    return;
                }
                if (StateManager.getAGL() < chan->config.airStartLockoutAltitude) {
                    Log.noticeln("below alt threshold");
                    return;
                }
                if (StateManager.getVertVel() < chan->config.airStartLockoutVelocity) {
                    return;
                }
                // FIXME: this should probably use an estimate manager, which probably
                // estimates in doubles, as opposed to the packet, which is floats
                // and is also in quarternions
                StatusManager.readData([](const StatusPacket &pkt, void *p) {
                    auto chan = static_cast<PyroChannel*>(p);

                    const SixFloats& position = pkt.estimate.position;
                    const auto lockoutAngle = chan->config.airStartLockoutAngle;
                    if (abs(position.pitch) >= lockoutAngle || abs(position.yaw) >= lockoutAngle) {
                        return; // outside of lockout angle
                    }

                    // all conditions met
                    chan->startFiringDelay();
                }, chan);
            break;

            default:
            break;
        }
    }, (void*)&event.args.intArgs.eventArg1);
}

void PyroManagerClass::onApogee() {
    if (liftedOff() == false || armed() == false) {
        return;
    }
    forEachChanLock([](PyroChannel* chan, size_t i, PyroManagerClass *self, void *ctx) {
        if (chan->config.channelType == PyroChannelConfig::DROUGE_CHANNEL) {
            chan->firingStartMS = millis();
        }
    }, NULL);
    rwLock.Lock();
    apogeeDetected = true;
    rwLock.UnLock();
}

void PyroManagerClass::onLanding() {
    onDisarmed();
}

PyroChannelConfig::PyroChannelConfig() : channelType(PyroChannelConfig::DISABLED_CHANNEL), delaySeconds(0), mainAlt(0), burnoutNumber(0), airStartLockoutAltitude(0), airStartLockoutAngle(0), airStartLockoutVelocity(0) {
}

PyroManagerClass::PyroChannel::PyroChannel(uint8_t FetChannel, uint8_t contChannel) : fetChannel(FetChannel), contChannel(contChannel), continuity(false), firing(false), firingStartMS(0), delayStartMS(0) {
}

PyroManagerClass::PyroChannel::~PyroChannel() {
}

void PyroManagerClass::PyroChannel::stopFiring() {
    digitalWrite(fetChannel, LOW);
    delay(1); // let settle
    pinMode(fetChannel, INPUT);
    firing = false;
    firingStartMS = 0;
}

void PyroManagerClass::PyroChannel::startFiring() {
    pinMode(fetChannel, OUTPUT);
    digitalWrite(fetChannel, HIGH);
    delayStartMS = 0;
    firing = true;
    firingStartMS = millis();
    Log.noticeln("firing channel %d (%s)", chanNum, chanTypeToString(config.channelType));
    PyroManager.event.eventType = Event::PYRO_FIRE_EVENT;
    PyroManager.event.args.intArgs.eventArg1 = chanNum;
    EventManager.publishEvent(PyroManager.event);
}

inline void PyroManagerClass::PyroChannel::startFiringDelay() {
    delayStartMS = millis();
}

inline bool PyroManagerClass::PyroChannel::firingDelayExpired() const {
    return (delayStartMS != 0 && (delayStartMS + config.delaySeconds*10000 < millis()));
}

inline bool PyroManagerClass::PyroChannel::firingDurationExpired() const {
    return (firing && firingStartMS != 0 && (firingStartMS + pulseWidthMS < millis()));
}

inline bool PyroManagerClass::PyroChannel::checkContinuity() {
    auto newCont = (digitalRead(contChannel) != 0);
    if (newCont ^ continuity) { // if they are different
        auto pyroStatus = StatusManager.getPyroStatus();
        if (continuity) {  // continuity loss
            if (firing) {
                stopFiring();
            }
            pyroStatus.clearContinuity(chanNum);
            PyroManager.indicators.setPixelColor(chanNum, 0);
            PyroManager.indicators.show();
            PyroManager.event.eventType = Event::CONTINUITY_LOSS_EVENT;
            PyroManager.event.args.intArgs.eventArg1 = chanNum;
            EventManager.publishEvent(PyroManager.event);
        } else { // continuity established
            pyroStatus.setContinuity(chanNum);
            PyroManager.indicators.setPixelColor(chanNum, 128, 0, 0);
            PyroManager.indicators.show();
        }
        StatusManager.setPyroStatus(pyroStatus);
    }
    continuity = newCont;
    return continuity;
}

void PyroManagerClass::testFire(uint8_t chanNum) {
    if (chanNum > numPyroChannels) {
        return;
    }
    if (armed() == true) {
        return;
    }
    rwLock.Lock();
    pyroChannels[chanNum]->startFiring();
    rwLock.UnLock();
}


void PyroManagerClass::forEachChanNoLock(void(*fn)(PyroChannel* chan, size_t i, PyroManagerClass *self, void *ctx), void *ctx) {
    if (fn == NULL) {
        return;
    }
    for (size_t i = 0; i < numPyroChannels; i++) {
        auto chan = pyroChannels[i];
        fn(chan, i, this, ctx);
    }
}

void PyroManagerClass::forEachChanLock(void(*fn)(PyroChannel* chan, size_t i, PyroManagerClass *self, void *ctx), void *ctx) {
    rwLock.Lock();
    forEachChanNoLock(fn, ctx);
    rwLock.UnLock();
}

