#pragma once

#include <subsystem.h>
#include <Filters.h>
#include <Filters/MedianFilter.hpp>

class BoostDetectorClass : public BaseSubsystem {
    public:
        BoostDetectorClass();
        virtual ~BoostDetectorClass();
        Status setup();
        Status start();

    private:
        static constexpr int altThresh = 100 * 0.3048; // 100 feet in meters

        bool armed;
        bool allreadyLiftedOff;
        int gps_gnd_alt;
        int baro_gnd_alt;

        int last_gps;
        int last_baro[2];
        float last_acc;
        unsigned long firstGoingUp;
        bool goingUp;

        MedianFilter<10, int> filtGPSalt;
        MedianFilter<10, int> filtBaroAlt;
        MedianFilter<100, float> filtAcc;

        void detect();
};

extern BoostDetectorClass BoostDetector;