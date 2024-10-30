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
#include "ticker.h"

#include <ArduinoJson.h>

class StatusSpew : public TickableSubsystem {
  public:
    StatusSpew() {}
    virtual ~StatusSpew() {}
    virtual Status setup() {
      setStatus(BaseSubsystem::READY);
      return getStatus();
    }
  protected:
    virtual Status tick() {
      static JsonDocument json;

      StatusManager.readData([](const StatusPacket &pkt, void *arg) {
            auto j = (JsonDocument*)arg;
            j->set(pkt);
        }, &json);
        serializeJsonPretty(json, LogWriter);
        return getStatus();
    }
} statusSpew;



static TickableSubsystem *tickers10ms[] = {&BMI088Subsystem, NULL};
static Ticker Ticker10ms(tickers10ms, 10);

static TickableSubsystem *tickers100ms[] = {
  &GPSSubsystem, 
  &BaroSubystem, 
  &MagSubsystem,
  &PyroManager, 
  &StatusManager,
  NULL};
static Ticker Ticker100ms(tickers100ms, 100);

static TickableSubsystem *tickers100s[] = {&statusSpew, NULL};
static Ticker Ticker100s(tickers100s, 100*1000);

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
}


void setup() {  
  Serial.begin();
  Serial.println("Starting up....");
  LogWriter.addSerialPrinter();

  Log.noticeln("setting up...");
  SubsystemManager.setup();
  Ticker100ms.setup();
  Ticker10ms.setup();
  Ticker100s.setup();

  subsystemGlue();

  Log.noticeln("starting...");
  SubsystemManager.start();
  Ticker100ms.start();
  Ticker10ms.start();
  Ticker100s.start();

  Event startEv;
  startEv.eventType = Event::START_EVENT;
  EventManager.publishEvent(startEv);
  Log.noticeln("playing a song");

  SoundSubsystem.playSong(SoundSubsystemClass::IDK_SONG);
  while(SoundSubsystem.playing()) {
    delay(100);
  }

  Log.noticeln("setup done");
}

void loop() {
}
