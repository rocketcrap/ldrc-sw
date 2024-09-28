#pragma once

#include <subsystem.h>

/**
 * @brief Calls tick() on collection of TickableSubsystems periodically
 *
 */
class Ticker : public ThreadedSubsystem {
 public:
    /**
     * @brief Construct a new Ticker object
     *
     * @param _subsystems array of pointers to TickableSubsystems, terminated with nullptr
     * @param _intervalMS millisecond interval to call tick() on them
     */
    Ticker(TickableSubsystem** _subsystems, int _intervalMS);
    virtual ~Ticker() {}

    /**
     * @brief setup the Ticker
     *
     * @note to start the ticker, you must call start(), inherited from the ThreadedSubsystem
     *
     * @return BaseSubsystem::Status
     */
    BaseSubsystem::Status setup();

 protected:
    virtual int taskPriority() const;
    virtual void taskFunction(void *parameter);


 private:
    Ticker();
    TickableSubsystem** subsystems;
    int intervalMS;
};