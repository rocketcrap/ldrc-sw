#include "sound-subystem.h"
#include "pins.h"
#include "log.h"
#include "configmanager.h"
#include "statusmanager.h"

SoundSubsystemClass SoundSubsystem;

SoundSubsystemClass::SoundSubsystemClass() : buzzer(BUZZER, 0), isPlaying(false) {
    name = "sound";
    static BaseSubsystem* deps[] = {&LogWriter, &StatusManager, &ConfigManager, NULL};
    static SubsystemManagerClass::Spec spec(this, deps);
    SubsystemManager.addSubsystem(&spec);

    queue = xQueueCreateStatic(QUEUE_SIZE,
            sizeof(Songs),
            queueStorage,
            &staticQueue); 
}

SoundSubsystemClass::~SoundSubsystemClass() {
}

BaseSubsystem::Status SoundSubsystemClass::setup() {
    setStatus(BaseSubsystem::READY);
    return getStatus();   
}

bool SoundSubsystemClass::playing() {
    bool rc;
    rwLock.RLock();
    rc = isPlaying;
    rwLock.RUnlock();
    return rc;
}

void SoundSubsystemClass::taskFunction(void *parameter) {
    Songs song;
    while(1) {
        if (xQueueReceive(queue, &song, portMAX_DELAY) == pdPASS) {
            switch (song) {
                case IDK_SONG:
                    rwLock.Lock();
                    isPlaying = true;
                    playIDKSong();
                    rwLock.UnLock();
                    break;
            }
        }
    }
}

void SoundSubsystemClass::playIDKSong() {
    buzzer.tone(NOTE_C4, 250);        
    buzzer.tone(NOTE_D4, 250);
    buzzer.tone(NOTE_E4, 250);
    buzzer.tone(NOTE_F4, 250);
    buzzer.tone(NOTE_G4, 250);
    buzzer.tone(NOTE_A4, 250);
    buzzer.tone(NOTE_B4, 250);
    buzzer.tone(NOTE_C5, 250);
    delay(250);
    buzzer.tone(NOTE_C5, 250);
    buzzer.tone(NOTE_B4, 250);
    buzzer.tone(NOTE_A4, 250);
    buzzer.tone(NOTE_G4, 250);
    buzzer.tone(NOTE_F4, 250);
    buzzer.tone(NOTE_E4, 250);
    buzzer.tone(NOTE_D4, 250);
    buzzer.tone(NOTE_C4, 250);
    buzzer.noTone();
}

bool SoundSubsystemClass::playSong(Songs song) {
    bool rc = false;
    if (song == NONE_SONG) {
        goto out;
    }
    if (song > IDK_SONG) {
        goto out;
    }

    rc = true;
    xQueueSend(queue, (void*)&song, portMAX_DELAY);
 
out:
    return rc;
}
