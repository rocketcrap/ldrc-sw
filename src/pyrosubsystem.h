#pragma once

#include <stdint.h>
#include <ArduinoJson.h>

#include "eventmanager.h"
#include "subsystem.h"
#include "statusmanager.h"
#include "packet.h"
#include "pyrochannelconfig.h"
#include <Adafruit_DotStar.h>

// TODO: create failsafe support

/**
 * @brief PyroManager controls pyro channels. It loads from config and acts based upon events received from elsewhere to trigger pyro channels
 *
 */
class PyroManagerClass : public TickableSubsystem {
    public:
        static uint8_t constexpr maxPyroChannels = 3; // LDRCv3 has 3 channels

        PyroManagerClass();
        virtual ~PyroManagerClass();
        virtual Status setup();
        virtual Status tick();

        /**
         * @brief check if all configured channels have continuity
         *
         * @return true all configured channels have continuity
         * @return false not all configured channels have continuity
         */
        bool allConfiguredChannelsContinuity();

        /**
         * @brief fire a channel manually for a pop test, for example
         *
         * @param chanNum the channel number to fire
         */
        void testFire(uint8_t chanNum);

        class PyroChannel {
            public:
                PyroChannel(uint8_t FetChannel, uint8_t contChannel);
                virtual ~PyroChannel();

                PyroChannelConfig config;
                friend class PyroManagerClass;

            private:
                void stopFiring();
                void startFiring();
                void startFiringDelay();
                bool firingDelayExpired() const;
                bool firingDurationExpired() const;
                bool checkContinuity();

                // defined by board type compile time
                const uint8_t fetChannel; // IO pin to trigger
                const uint8_t contChannel; // IO pin to measure continuity

                // index in array, defined in constructor of outer class
                size_t chanNum;

                // state information
                bool continuity;
                bool firing; // if currently firing
                uint32_t firingStartMS; // millis() when firing state entered
                uint32_t delayStartMS; // millis() when firing delay state entered
        };

    private:
        static uint16_t constexpr pulseWidthMS = 500; // 1/2 second
        static constexpr auto numLED = 3;

        uint8_t numPyroChannels;
        PyroChannel *pyroChannels[maxPyroChannels];

        bool liftOffDetected;
        bool apogeeDetected;
        bool pyroArmed;

        Adafruit_DotStar indicators;

        Event event;

        void forEachChanNoLock(void(*fn)(PyroChannel* chan, size_t i, PyroManagerClass *self, void *ctx), void *ctx);
        void forEachChanLock(void(*fn)(PyroChannel* chan, size_t i, PyroManagerClass *self, void *ctx), void *ctx);

        bool liftedOff() const;
        bool postApogee() const;
        bool armed() const;

        void onArmed();
        void onDisarmed();
        void onLiftOff();
        void onBurnout(const Event &event);
        void onApogee();
        void onLanding();

        void tickContinuityChanges();
        void tickFiringDelay();
        void tickFiringDuration();
        void tickAltitudeTrigger();
};

extern PyroManagerClass PyroManager;