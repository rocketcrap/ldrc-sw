#pragma once

#include "subsystem.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <packet.h>


//uncomment to make NMEA sentences appear in serial log
//#define GPS_SPEW

class GPSSubsystemClass : public TickableSubsystem, public DataProvider<GPSFix> {
    public:
        GPSSubsystemClass();
        virtual ~GPSSubsystemClass();
        BaseSubsystem::Status setup();

        GPSFix getFix() const;

        BaseSubsystem::Status tick();

        bool lowPowerMode();

    private:
        SFE_UBLOX_GNSS gps;
        uint32_t gpsLoopMillis;
        uint32_t positioningMillis;
        bool noFixYet;

        void setRTC();
};

extern GPSSubsystemClass GPSSubsystem;