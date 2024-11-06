#pragma once

#include <subsystem.h>
#include <Filters/MedianFilter.hpp>

/**
 * @brief Calls tick() on collection of TickableSubsystems periodically
 *
 */
class Ticker : public ThreadedSubsystem {
   public:
      /**
       * @brief Construct a new Ticker object
       *
       * @param subsystems array of pointers to TickableSubsystems, terminated with nullptr
       * @param intervalMS millisecond interval to call tick() on them
       * @param name name of this ticker subsystem
       * @param priority the priority of this ticker subsystem
       */
      Ticker(TickableSubsystem** subsystems, int intervalMS, const char* name = "unnamed ticker", int priority=1);
      virtual ~Ticker() {}

      /**
       * @brief start the ticker
       *
       * @note Overridden to restart all tickable subsystems when starting from stop
       *
       * @return Status RUNNING
       */
      Status start();

      /**
       * @brief setup the Ticker
       *
       * @note to start the ticker, you must call start(), inherited from the ThreadedSubsystem
       *
       * @return BaseSubsystem::Status
       */
      BaseSubsystem::Status setup();

      /**
       * @brief get the period of this ticker
       *
       * @return int the period in MS
       */
      int period() const;

      /**
       * @brief Set the period of this ticker
       *
       * @param period the period in ms
       */
      void setPeriod(int period);

      /**
       * @brief Get the percentage this ticker is "Busy"
       *
       * @return int 0-100 percent
       */
      int getPercentBusy() const;

      /**
       * @brief enter low power mode
       *
       * @details Stop the ticking and call lowPowerMode on everything
       *
       * @return true
       * @return false
       */
      bool lowPowerMode();

   protected:
      Ticker() = delete;
      virtual int taskPriority() const;
      virtual void taskFunction(void *parameter);

   private:
      static constexpr auto MAX_DEPS = 8;

      SubsystemManagerClass::Spec spec;
      BaseSubsystem* deps[MAX_DEPS+1];
      TickableSubsystem** subsystems;
      int intervalMS;
      int priority;
      MedianFilter<10, uint32_t> durationFilter;
      uint32_t medianDuration;
};