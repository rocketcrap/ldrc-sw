#pragma once

#include <subsystem.h>
#include "packet.h"
#include <Filters.h>
#include <Filters/MedianFilter.hpp>
#include <CircularBuffer.hpp>
#include <Differentiator.h>

// Still WIP
// DONE: StateManager as source of AGL altitude
// DONE: StateManager as a source of vert velocity
// DONE: convert pyro to be armed/disarmed via state manager
// FIXME: Battery measurement or kick out battery failing state
// FIXME: all systems go check
// FIXME: pointing up

class StateManagerClass : public BaseSubsystem {
    public:
        StateManagerClass();
        virtual ~StateManagerClass();
        Status setup();
        Status start();

        /**
         * @brief Get current State
         *
         * @return Packet::State
         */
        Packet::State getState() const;

        /**
         * @brief test if Arming is possible. Sets error retrievable with armError()
         *
         * @return true can be armed
         * @return false cannot be armed. See error with armError()
         */
        bool canArm();

        /**
         * @brief arm LDRC, enabling transition to flight events
         *
         * @return true arming success
         * @return false cannot be armed. See error with armError()
         */
        bool arm();

        /**
         * @brief return error set with arm() or canArm()
         *
         * @return const char* const error string
         */
        const char * const armError() const;

        /**
         * @brief disarm
         *
         * @note void, but disarm does nothing if not armed or in flight states
         *
         */
        void disarm();

        /**
         * @brief get estimate altitude over ground (at launch point)
         *
         * @return int altitude in meters above ground.
         */
        int getAGL() const;

        /**
         * @brief Get the vertical velocity according to filtered baro data
         *
         * @return int vertical velocity in meters per second
         */
        int getVertVel() const;

    private:
        static constexpr auto FT_PER_METER = 0.3048f;
        static constexpr auto G = 9.8f;

        static constexpr int liftOffAltThresh = 100 * FT_PER_METER; // threshold to detect liftoff
        static constexpr auto boostThresh = 2 * G; // threshold to establish boost/liftoff
        static constexpr auto vertVelThresh = 100 * FT_PER_METER; // threshold to stablish boost

        static constexpr auto chuteVelThresh = 200 * FT_PER_METER; // threshold of under chute/lawn dart
        static constexpr auto chuteAccThresh = 3.0f; // threshold of under chute/lawn dart


        static constexpr auto machLockoutTrigger = 800 * FT_PER_METER; // mach lockout trigger
        static constexpr auto machLockoutRelease = 100 * FT_PER_METER; // mach lockout lower threshold

        static constexpr auto coastThresh = 0.25 * G; // 1/4 G is thresh to declare coast

        static constexpr auto lostThreshold = 2*3600; // how long before your rocket is "lost"

        const char * armingError;

        // last gps recorded alt - used in liftoff detection
        int last_gps;

        template <typename T>
        struct Reading {
            Reading<T>(T _value) : value(_value), time(millis()) {}
            Reading<T>(){}
            T value;
            unsigned long time;
        };

        CircularBuffer<Reading<float>, 10> baroReadings;

        MedianFilter<10, int> filtGPSalt;
        MedianFilter<10, float> filtBaroAlt;
        MedianFilter<100, float> filtAcc;

        // barometric vertical velocity
        Differentiator vel;
        float vertVel;

        // barometric vertical acceleration
        Differentiator acc;
        float vertAcc;

        Packet::State state;

        // last magnitude of acc from imu
        float last_acc;

        // used in liftoff detection
        unsigned long firstGoingUp;
        bool goingUp;

        // measured ground altitude for calculated AGL
        int gps_gnd_alt;
        int baro_gnd_alt;

        // used in coast detection
        uint8_t burnoutCount;

        // used in apogee detection
        bool lockOutActive;
        long lockOutStarted;
        Reading<float> maxAlt;

        // used for lost rocket detection
        int32_t landed_time;

        void detect();
        bool detectLiftoff();
        bool detectBoost();
        bool detectCoast();
        bool detectApogee();
        bool detectUnderChute();
        bool detectLawnDart();
        bool detectTouchdown();
        bool detectPowerFail();
        bool detectLost();
};

extern StateManagerClass StateManager;