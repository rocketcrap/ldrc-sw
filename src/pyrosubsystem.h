#pragma once

#include <stdint.h>
#include <ArduinoJson.h>

#include "eventmanager.h"
#include "subsystem.h"
#include "statusmanager.h"
#include "packet.h"
#include "pyrochannelconfig.h"



/**
 * @brief PyroManager controls pyro channels. It loads from config and acts based upon events received from elsewhere to trigger pyro channels
 *
 */
class PyroManagerClass : public TickableSubsystem {
    public:
        static uint8_t constexpr maxPyroChannels = 8; // who could need more than eight?

        PyroManagerClass();
        virtual ~PyroManagerClass();
        virtual Status setup();
        virtual Status tick();

        /**
         * @brief return if PyroManager is armed
         *
         * @return true if armed
         * @return false if not armed
         */
        bool armed() const;

        /**
         * @brief arm the PryoManager
         *
         */
        void arm();

        /**
         * @brief disarm the PyroManager
         *
         */
        void disarm();

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

        static uint32_t constexpr armedFlagValue = 0xDEADBEEF;
        static size_t constexpr numArmFlags = 4;

        void forEachChanNoLock(void(*fn)(PyroChannel* chan, size_t i, PyroManagerClass *self, void *ctx), void *ctx);
        void forEachChanLock(void(*fn)(PyroChannel* chan, size_t i, PyroManagerClass *self, void *ctx), void *ctx);


        bool liftedOff() const;
        bool postApogee() const;
        bool flightComputer() const;

        void onLiftOff(const Event &event);
        void onBurnout(const Event &event);
        void onApogee(const Event &event);
        void onLanding(const Event &event);

        void tickContinuityChanges();
        void tickFiringDelay();
        void tickFiringDuration();
        void tickAltitudeTrigger();

        uint8_t numPyroChannels;
        PyroChannel *pyroChannels[maxPyroChannels];

        bool liftOffDetected;
        bool apogeeDetected;
        bool amFlghtComputer;

        uint32_t armFlag[4];

        Event event;
};

extern PyroManagerClass PyroManager;