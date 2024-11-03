#include "statemanager.h"
#include "eventmanager.h"
#include "gps-subsystem.h"
#include "baro-subsystem.h"
#include "bmi088-subsystem.h"
#include "mag-subsystem.h"
#include "log.h"
#include "pyrosubsystem.h"
#include <AH/Math/Vector.hpp>

StateManagerClass StateManager;

StateManagerClass::StateManagerClass() : state(Packet::INIT), vel(TimeStep(0.1)), acc(TimeStep(0.1)), burnoutCount(0) {
      name = "state manager";
      strncpy(armingError, "", sizeof(armingError));

      //FIXME: more deps
      static BaseSubsystem* deps[] = {&BaroSubystem, &GPSSubsystem, &BMI088Subsystem, &EventManager, &LogWriter, NULL};
      static SubsystemManagerClass::Spec spec(this, deps);
      SubsystemManager.addSubsystem(&spec);
}

StateManagerClass::~StateManagerClass() {
}

BaseSubsystem::Status StateManagerClass::setup() {
      state = Packet::DISARMED;

      // GPS
      GPSSubsystem.registerCallback([](const GPSFix& fix, void *arg) {
            auto self = static_cast<StateManagerClass*>(arg);
            self->rwLock.Lock();

            if (fix.fixType == 3) { // must have 3d fix
                  self->last_gps = self->filtGPSalt(fix.altitude);
            }
            self->detect();

            self->rwLock.UnLock();
      }, this);

      // Barometer
      BaroSubystem.registerCallback([](const BarometerData& baro, void *arg) {
            auto self = static_cast<StateManagerClass*>(arg);
            self->rwLock.Lock();

            const auto filteredValue = self->filtBaroAlt(baro.altitude);
            self->baroReadings.push(filteredValue);

            const auto vel = self->vel.step(filteredValue);
            const auto acc = self->acc.step(vel);
            self->vertVel = vel;
            self->vertAcc = acc;

            self->detect();

            self->rwLock.UnLock();
      }, this);

      // IMU
      BMI088Subsystem.registerCallback([](const SixFloats& data, void *arg) {
            auto self = static_cast<StateManagerClass*>(arg);
            self->rwLock.Lock();

            // YOLO fuck it, let's just see if we're accelerating overall
            self->last_acc = self->filtAcc(Vec3f(data.x, data.y, data.z).norm());

            // we don't call detect() here b/c this is sampled every 10ms and barometer
            // is every 100ms

            self->rwLock.UnLock();
      }, this);

      setStatus(BaseSubsystem::READY);
      return getStatus();
}

BaseSubsystem::Status StateManagerClass::start() {
      setStatus(BaseSubsystem::RUNNING);
      return getStatus();
}

Packet::State StateManagerClass::getState() const {
      rwLock.RLock();
      auto rc = state;
      rwLock.RUnlock();
      return rc;
}

bool StateManagerClass::canArm() {
      bool rc = false;

      rwLock.RLock();

      if (state != Packet::DISARMED) {
            strncpy(armingError, "refusing to arm b/c not DISARMED", sizeof(armingError));
            Log.errorln(armingError);
            goto out;
      }

      SubsystemManager.iterateSubsystems([](const BaseSubsystem *it, void *args) {
            auto self = static_cast<StateManagerClass*>(args);
            const auto status = it->getStatus();
            if (status != BaseSubsystem::RUNNING && status != BaseSubsystem::STOPPED) {
                  snprintf(self->armingError, sizeof(self->armingError), 
                        "refusing to arm b/c subsystem: %s in state %s", 
                        it->name, it->statusString());
            }

      }, this);

      if (!PyroManager.allConfiguredChannelsContinuity()) {
            strncpy(armingError, "not all configured channels have continuity", sizeof(armingError));
            Log.errorln(armingError);
            goto out;
      }

      // TODO: pointing up

      // TODO: not moving

out:
      return rc;
      rwLock.RUnlock();

}

const char * StateManagerClass::armError() const {
      rwLock.RLock();
      auto rc = armingError;
      rwLock.RUnlock();
      return rc;
}

bool StateManagerClass::arm() {
      bool rc = false;

      if (canArm()) {
            rwLock.Lock();

            // establish ground levels
            gps_gnd_alt = last_gps;
            baro_gnd_alt = baroReadings.last().value;

            rc = true;
            Log.noticeln("arming");
            state = Packet::ARMED;
            EventManager.publishEvent(Event::ARM_EVENT);

            rwLock.UnLock();
      }
      return rc;
}

void StateManagerClass::disarm() {
      rwLock.Lock();
      // can only disarm if armed
      if (state == Packet::ARMED) {
            Log.noticeln("disarmed");
            state = Packet::DISARMED;
      }
      rwLock.UnLock();
      EventManager.publishEvent(Event::DISARM_EVENT);
}

int StateManagerClass::getAGL() const {
      // TODO: Kalman with GPS- Baro going to stop giving you good data over about 50k
      rwLock.RLock();
      const int rc = roundf(baroReadings.last().value - baro_gnd_alt);
      rwLock.RUnlock();
      return rc;
}

int StateManagerClass::getVertVel() const {
      rwLock.RLock();
      const int rc = roundf(vertVel);
      rwLock.RUnlock();
      return rc;
}

void StateManagerClass::detect() {
      switch (state) {
            case Packet::ARMED:
                  detectLiftoff();
                  break;
            case Packet::BOOST:
                  detectCoast();
                  break;
            case Packet::COAST:
                  if (!detectBoost()) {
                       detectApogee();
                  }
                  break;
            case Packet::APOGEE:
                  if (!detectUnderChute()) {
                        detectLawnDart();
                  }
                  break;
            case Packet::LAWN_DART:
                  if (!detectUnderChute()) {
                        detectTouchdown();
                  }
                  break;
            case Packet::UNDER_CHUTE:
                  detectTouchdown();
                  break;
            case Packet::TOUCHDOWN:
                  if (!detectLost()) {
                        detectPowerFail();
                  }
                  break;
            case Packet::LOST:
                  detectPowerFail();
                  break;
            case Packet::POWER_FAIL:
                  // TODO: ???
                  break;
      }
}

bool StateManagerClass::detectLiftoff() {
      bool rc = false;
      // liftoff detection:
      //    "going up" trigger
      //       for last 100ms, we have been:
      //       going up according to baro and accelerometers
      //
      //   GPS and barometer agree that we've ascended at least 100' and we're "going up"

      const auto currentBaro = baroReadings.last().value;
      const auto priorBaro = baroReadings.first().value;
      const auto now = millis();
      if (currentBaro > priorBaro && last_acc > boostThresh) { // we are going up
            if (!goingUp) {
                  firstGoingUp = now;
                  Log.noticeln("boost Detector: we are now going up");
            }
            goingUp = true;
      } else { // we are not going up
            if (goingUp) {
                  Log.noticeln("boost Detector: we are no longer going up");
            }
            goingUp = false;
      }
      const auto gpsSaysUp = gps_gnd_alt + liftOffAltThresh < last_gps;
      const auto baroSaysUp = baro_gnd_alt + liftOffAltThresh < currentBaro;
      const auto ago100ms = firstGoingUp + 100 < now;
      if (goingUp && ago100ms && gpsSaysUp && baroSaysUp) {
            state = Packet::BOOST;
            Log.noticeln("liftoff!");
            EventManager.publishEvent(Event::LIFTOFF_EVENT);
            rc = true;
      }
      return rc;
}

bool StateManagerClass::detectCoast() {
      bool rc = false;
      if (last_acc < coastThresh && vertVel > vertVelThresh) {
            burnoutCount++;
            state = Packet::COAST;
            Log.noticeln("Burnout number %d", burnoutCount);
            Event ev;
            ev.eventType = Event::BURNOUT_EVENT;
            ev.args.intArgs.eventArg1 = burnoutCount;
            EventManager.publishEvent(ev);
            rc = true;
      }
      return rc;
}

bool StateManagerClass::detectBoost() {
      bool rc = false;

      if ((last_acc > boostThresh) && (vertVel > vertVelThresh)) {
            state = Packet::BOOST;
            Log.noticeln("boost");
            EventManager.publishEvent(Event::AIRSTART_EVENT);
            rc = true;
      }

      return rc;
}

bool StateManagerClass::detectApogee() {
      const auto now = millis();

      bool rc = false;

      const auto currentReading = baroReadings.last();
      if (maxAlt.value > currentReading.value) {
            maxAlt = currentReading;
      }

      if (vertVel > machLockoutTrigger) {
            Log.noticeln("Mach lockout started");
            lockOutActive = true;
            lockOutStarted = now;
            goto out;
      }
      // If you're below lockout thresh and have been for over 1 second, then disable lockout
      if (lockOutActive && vertVel < machLockoutRelease && (now - lockOutStarted) > 1000 ) {
            Log.noticeln("Mach lockout ended");
            lockOutActive = false;
      }

      if (!lockOutActive &&  currentReading.value < maxAlt.value && now - maxAlt.time > 1000) {
            state = Packet::APOGEE;
            Log.noticeln("apogee!!!");
            Event ev;
            ev.eventType = Event::APOGEE_EVENT;
            ev.args.intArgs.eventArg1 = maxAlt.value;
            EventManager.publishEvent(ev);
            rc = true;
      }

out:
      return rc;
}

bool StateManagerClass::detectUnderChute() {
      bool rc = false;

      if (vertVel < chuteVelThresh && abs(vertAcc) < chuteAccThresh) {
            rc = true;
            // There is no event for under CHUTE
            state = Packet::UNDER_CHUTE;
            Log.noticeln("chute detected");
      }

      return rc;
}

bool StateManagerClass::detectLawnDart() {
      bool rc = false;
      if (vertVel > chuteVelThresh && abs(vertAcc) > chuteAccThresh) {
            rc = true;
            state = Packet::LAWN_DART;
            Log.noticeln("Lawn dart!");
            EventManager.publishEvent(Event::LAWN_DART_EVENT);
      }

      return rc;
}

bool StateManagerClass::detectTouchdown() {
      bool rc = false;

      if (vertVel < 1 && vertAcc < 1 && last_acc < G) {
            rc = true;
            state = Packet::TOUCHDOWN;

            struct timeval tv;
            gettimeofday(&tv, NULL);
            landed_time = tv.tv_sec;

            Log.noticeln("touchdown at %d", landed_time);

            EventManager.publishEvent(Event::LANDING_EVENT);
      }

      return rc;
}

bool StateManagerClass::detectPowerFail() {
      bool rc = false;

      // FIXME: read the condition
      // The actual measurement is in statusmanager
      // but that seems kinda meh
      // but is power fail actually a state?
      // couldn't status manager just throw out the powerfail event??
      if (false) {
            rc = true;
            state = Packet::POWER_FAIL;
            EventManager.publishEvent(Event::LOW_BATTERY_EVENT);
            // FIXME: include batt voltage in event
      }

      return rc;
}

bool StateManagerClass::detectLost() {
      bool rc = false;
      struct timeval tv;

      gettimeofday(&tv, NULL);
      if (tv.tv_sec - landed_time > lostThreshold) {
            state = Packet::LOST;
            Log.noticeln("lost rocket!");
            rc = true;
            EventManager.publishEvent(Event::LOST_ROCKET_EVENT);
      }

      return rc;
}