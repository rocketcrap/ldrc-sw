#pragma once

#include "subsystem.h"
#include "configmanager.h"


class WifiSubsystemClass : public BaseSubsystem {
    public:
        WifiSubsystemClass();
        virtual ~WifiSubsystemClass();

        virtual Status setup();
        Status start();
        Status stop();

    private:
        ConfigData::WIFIConfig wifiConfig;
};

extern WifiSubsystemClass WifiSubsystem;