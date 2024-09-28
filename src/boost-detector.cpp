#include "boost-detector.h"
#include "eventmanager.h"
#include "gps-subsystem.h"
#include "baro-subsystem.h"
#include "bmi088-subsystem.h"
#include "log.h"


#include <AH/Math/Vector.hpp>

BoostDetectorClass BoostDetector;

BoostDetectorClass::BoostDetectorClass() : armed(false), allreadyLiftedOff(false) {
      name = "boost detector";
      static BaseSubsystem* deps[] = {&BaroSubystem, &GPSSubsystem, &BMI088Subsystem, &EventManager, &LogWriter, NULL};
      static SubsystemManagerClass::Spec spec(this, deps);
      SubsystemManager.addSubsystem(&spec);
}

BoostDetectorClass::~BoostDetectorClass() {
}

BaseSubsystem::Status BoostDetectorClass::setup() {
  // arm and disarm
  EventManager.subscribe([](const Event& ev, void* arg) {
        auto self = static_cast<BoostDetectorClass*>(arg);
        self->rwLock.Lock();

        if (ev.eventType == Event::ARM_EVENT) {
            self->armed = true;
            self->gps_gnd_alt = self->last_gps;
            self->baro_gnd_alt = self->last_baro[1];
        } else {
            self->armed = false;
        }
        self->rwLock.UnLock();
  }, Event::ARM_EVENT| Event::DISARM_EVENT, this);

  // GPS
  GPSSubsystem.registerCallback([](const GPSFix& fix, void *arg) {
      auto self = static_cast<BoostDetectorClass*>(arg);
      self->rwLock.Lock();

      if (fix.fixType == 3) { // must have 3d fix
          self->last_gps = self->filtGPSalt(fix.altitude);
      }
      self->detect();

      self->rwLock.UnLock();
  }, this);

  // Barometer
  BaroSubystem.registerCallback([](const BarometerData& baro, void *arg) {
      auto self = static_cast<BoostDetectorClass*>(arg);
      self->rwLock.Lock();

      self->last_baro[0] = self->last_baro[1];
      self->last_baro[1] = self->filtBaroAlt(baro.altitude);
      self->detect();

      self->rwLock.UnLock();
  }, this);

  // IMU
  BMI088Subsystem.registerCallback([](const SixFloats& data, void *arg) {
      auto self = static_cast<BoostDetectorClass*>(arg);
      self->rwLock.Lock();

      // YOLO fuck it, let's just see if we're accelerating overall
      self->last_acc =  self->filtAcc(Vec3f(data.x, data.y, data.z).norm());
      self->detect();

      self->rwLock.UnLock();
  }, this);
  setStatus(BaseSubsystem::READY);
  return getStatus();
}

BaseSubsystem::Status BoostDetectorClass::start() {
      setStatus(BaseSubsystem::RUNNING);
      return getStatus();
}

void BoostDetectorClass::detect() {
      // Boost detection:
      //   must be armed (which requires GPS fix)
      //    "going up" trigger
      //       for last 100ms, we have been:
      //       going up according to baro and accelerometers
      //
      //   GPS and barometer agree that we've ascended at least 100' and we're "going up"
      if (armed && !allreadyLiftedOff) {
            if (last_baro[1] > last_baro[0] && last_acc > 2) { // we are going up
                  if (!goingUp) {
                        firstGoingUp = millis();
                        Log.noticeln("liftoff Detector: we are now going up");
                  }
                  goingUp = true;
            } else { // we are not going up
                  if (goingUp) {
                        Log.noticeln("liftoff Detector: we are no longer going up");
                  }
                  goingUp = false;
            }
            auto gpsThresh = gps_gnd_alt + altThresh;
            auto baroThresh = baro_gnd_alt + altThresh;
            auto ago100ms = firstGoingUp + 100 < millis();
            if (goingUp && (ago100ms) && (last_gps > gpsThresh) && (last_baro[1] > baroThresh)) {
                  Event ev;
                  ev.eventType = Event::LIFTOFF_EVENT;
                  allreadyLiftedOff = true;
                  EventManager.publishEvent(ev);
            }
      }
}