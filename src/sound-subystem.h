#pragma once

#include <subsystem.h>
#include <ToneESP32.h>

class SoundSubsystemClass : public ThreadedSubsystem {
public:
    SoundSubsystemClass();
    virtual ~SoundSubsystemClass();
    BaseSubsystem::Status setup();
    enum Songs {
      NONE_SONG,
      IDK_SONG  
    };
    bool playSong(Songs song);
    bool playing();

protected:
    void taskFunction(void *parameter);

private:
    static constexpr int QUEUE_SIZE = 8;

    bool isPlaying;
    ToneESP32 buzzer;
    StaticQueue_t staticQueue;
    QueueHandle_t queue;
    uint8_t queueStorage[QUEUE_SIZE * sizeof(Songs)];

    // TODO: import some songs from https://github.com/robsoncouto/arduino-songs
    void playIDKSong();
};

extern SoundSubsystemClass SoundSubsystem;