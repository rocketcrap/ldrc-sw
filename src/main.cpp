#include "pins.h"
#include <Arduino.h>
#include "subsystem.h"
#include "log.h"
#include "gps-subsystem.h"
#include "baro-subsystem.h"
#include "mag-subsystem.h"
#include "statusmanager.h"
#include "sound-subystem.h"
#include "pyrosubsystem.h"
#include "bmi088-subsystem.h"
#include "eventmanager.h"
#include "websubsystem.h"
#include "ticker.h"

#include <ArduinoJson.h>

class StatusSpew : public TickableSubsystem {
  public:
    StatusSpew() {}
    virtual ~StatusSpew() {}
    Status setup() {
      setStatus(BaseSubsystem::READY);
      return getStatus();
    }
    int period() const {
      return 10*1000;
    }
  protected:
    Status tick() {
      static JsonDocument json;

      StatusManager.readData([](const StatusPacket &pkt, void *arg) {
            auto j = (JsonDocument*)arg;
            j->set(pkt);
        }, &json);
        serializeJsonPretty(json, LogWriter);
        return getStatus();
    }
} statusSpew;


static TickableSubsystem *SPITickers[] = {&BMI088Subsystem, NULL};
static Ticker SPITicker(SPITickers, 10, "SPI Ticker", 2);

static TickableSubsystem *slowerTickers[] = {
  &GPSSubsystem, 
  &BaroSubystem, 
  &MagSubsystem,
  &PyroManager, 
  &StatusManager,
  &WebSubsystem,
  NULL};
static Ticker slowerTicker(slowerTickers, 100);

static TickableSubsystem *verySlowTickers[] = {&statusSpew, NULL};
static Ticker verySlowTicker(verySlowTickers, 100*1000);

// just publish to StatusManager
static void subsystemGlue() {
  GPSSubsystem.registerCallback([](const GPSFix& f, void* args){
    StatusManager.setGPSFix(f);
  }, NULL);
  BaroSubystem.registerCallback([](const BarometerData& d, void* args){
    StatusManager.setBarometerData(d);
  }, NULL);
  BMI088Subsystem.registerCallback([](const SixFloats& f, void* args){
    StatusManager.setIMUData(f);
  }, NULL);
  EventManager.subscribe([](const Event &ev, void* arg) {
    switch(ev.eventType) {
      case Event::START_EVENT:
      SPITicker.setPeriod(100);
      slowerTicker.setPeriod(1000);
      break;

      case Event::ARM_EVENT:
      // full speed
      SPITicker.setPeriod(10);
      slowerTicker.setPeriod(100);
      break;

      case Event::DISARM_EVENT:
      // Slow down, but still able to function
      SPITicker.setPeriod(100); 
      slowerTicker.setPeriod(1000);
      break;

      case Event::LANDING_EVENT:
      SPITicker.lowPowerMode();
      slowerTicker.lowPowerMode();
      break;
    }
  }, Event::ALL_EVENT_MASK, NULL);
}


void setup() {  
  Serial.begin();
  Serial.println("Starting up....");
  LogWriter.addSerialPrinter();

  Log.noticeln("setting up...");
  SubsystemManager.setup();

  subsystemGlue();

  Log.noticeln("starting...");
  SubsystemManager.start();

  EventManager.publishEvent(Event::START_EVENT);
  Log.noticeln("playing a song");

  SoundSubsystem.playSong(SoundSubsystemClass::IDK_SONG);
  while(SoundSubsystem.playing()) {
    delay(100);
  }

  Log.noticeln("setup done");
}

void loop() {
}
