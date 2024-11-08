#pragma once

#include <Arduino.h>
#include "rwlock.h"

/**
 * @brief BaseSubsystem is the base class of all subsystems.
 *
 */
class BaseSubsystem {
 public:
    enum Status {
        INIT,       ///< The initial state
        READY,      ///< After successfully running setup()
        FAULT,      ///< If a fault has occurred
        RUNNING,    ///< Subsystem is running normally
        STOPPED     ///< Subsystem is stopped normally
    };

   /**
    * @brief get a string representation of status
    *
    * @return const char* const string representation of status
    */
   const char * const statusString() const;

    /**
     * @brief The human readable name of the subsystem
     *
     */
    const char *name;

    /**
     * @brief override this method in subclass to setup the subsystem
     *
     * @return Status
     */
    virtual Status setup() = 0;

    /**
     * @brief override in subclass. Meaningful in only some cases
     *
     * @return Status
     */
    virtual Status start() = 0;

   /**
    * @brief stop this subsystem
    *
    * @return Status STOPPED or FAULT
    */
    virtual Status stop();

    /**
     * @brief Get the Status of the subsystem
     *
     * @return Status
     */
    Status getStatus() const;

   /**
    * @brief enter into a lower power state if possible
    *
    * @note default implementation just returns false
    *
    * @return true entered into low power state
    * @return false did not enter into low power state
    */
    virtual bool lowPowerMode();

    // to get access to name
    friend class SubsystemManagerClass;

 protected:
    BaseSubsystem();
    virtual ~BaseSubsystem();

    /**
     * @brief Set the Status of the subsystem
     *
     * @param newStatus
     */
    void setStatus(Status newStatus);

    /**
     * @brief Inner status attribute. Do not set/get directly, use setter/getter
     *
     */
    Status status;

    /**
     * @brief thread safe read/write lock w/ 8 slots
     *
     */
    mutable ReadWriteLock rwLock;
};

/**
 * @brief Inherit from this class if your subsystem needs to be periodically called
 *
 */
class TickableSubsystem : public BaseSubsystem {
 public:
    virtual ~TickableSubsystem();

    /**
     * @brief override this method to implement specific subsystem functionality
     *
     * @return Status
     */
    virtual Status tick() = 0;

    virtual Status start();
};

/**
 * @brief Inherit from this subsystem if your subsystem requires a thread to run
 *
 */
class ThreadedSubsystem : public BaseSubsystem {
 public:
    ThreadedSubsystem();
    virtual ~ThreadedSubsystem();

    /**
     * @brief start the thread
     *
     * @return Status
     */
    virtual Status start();

    /**
     * @brief stop the thread
     *
     * @return Status
     */
    virtual Status stop();

 protected:
    /**
     * @brief override to return the task priority of your choosing. Defaults to tskIDLE_PRIORITY
     *
     * @return int
     */
    virtual int taskPriority() const;

    /**
     * @brief override to return a parameter to taskFunction(). Defaults to nullptr
     *
     * @return void* const
     */
    virtual void * const taskParameter();

    /**
     * @brief implement to provide a task function for your thread.
     *
     * @note implementations should implement an infinit inner loop
     *
     * @param parameter
     */
    virtual void taskFunction(void *parameter) = 0;

   /**
    * @brief which core this task should run on
    *
    * @return int core number to run on
    */
   virtual int core();

    /**
     * @brief TaskHandle for the thread of this subsystem
     *
     */
    TaskHandle_t taskHandle;


 private:
    static const auto STACK_SIZE = 4096; // 4K ought to be enough for anyone, right?
    StaticTask_t taskBuffer;
    StackType_t taskStack[STACK_SIZE];
};

/**
 * @brief DataProvider is designed to provide subscribe read primitives
 *
 * @tparam T type of underlying data to provide
 */
template<class T>
class DataProvider {
   public:
      typedef void(DataFn)(const T &, void *args);

      /**
       * @brief Construct a new Data Thing object
       *
       * @note Use this constructor in your subclass's constructor as DataThing<klass>(rwLock)
       *
       * @param locker a ReadWriteLocker to lock
       */
      DataProvider(ReadWriteLock &locker) : lock(locker), numCallbacks(0) {}

      virtual ~DataProvider() {}

      /**
       * @brief register a callback to be called when Data changes
       *
       * @param fn a function to be called with const reference to data
       * @param args additional arguments to be call function with
       */
      void registerCallback(DataProvider<T>::DataFn fn, void *args) {
         callback cb  {
            .args = args,
            .fn = fn
         };

         lock.Lock();

         if (numCallbacks == MAX_CALLBACKS) {
            //Log.errorln("Tried to add beyond %d callbacks", MAX_CALLBACKS);
            goto out;
         }
         callbacks[numCallbacks] = cb;
         numCallbacks++;

      out:
         lock.UnLock();
      }

      /**
       * @brief read underlying data
       *
       * note that fn will called with thing rlocked. fn that calls write operation on class will result in deadlock
       *
       * @param fn a function to be called with const reference to data
       * @param args additional arguments to be call function with
       */
      void readData(DataProvider<T>::DataFn fn, void *args) const {
         lock.RLock();
         fn(data, args);
         lock.RUnlock();
      }

      /**
       * @brief access data w/ read/write reference
       *
       * @param fn callback with write access to data.
       * @param args arg to pass to fn
       */
      void accessData(void(fn)(T &data, void *args), void *args) {
         lock.Lock();
         fn(data, args);
         lock.UnLock();
         callCallbacks();
      }

   protected:
      /**
       * @brief call this method to call callbacks registered with registerCallback()
       *
       */
      virtual void callCallbacks() {
         onUpdate(); // invoke hook if defined

         lock.RLock();

         for (auto i = 0; i < numCallbacks; i++) {
            callback cb = callbacks[i];
            cb.fn(data, cb.args);
         }

         lock.RUnlock();
      }

      /**
       * @brief You can override this function to have an internal hook prior to calling the callbacks
       *
       */
      virtual void onUpdate() {}

      /**
       * @brief The actual data itself
       *
       */
      T data;

   private:
      static constexpr size_t MAX_CALLBACKS = 8;

      DataProvider() = delete;
      DataProvider(const DataProvider& other) = delete;

      ReadWriteLock &lock;
      int numCallbacks;

      struct callback {
         void *args;
         void (*fn)(const T&, void*);
      };
      callback callbacks[MAX_CALLBACKS];
};

/**
 * @brief SubsystemManager allows for consistent starting of subsystems without cluttering main
 *
 */
class SubsystemManagerClass : public BaseSubsystem {
public:
   typedef void(SubsystemFn)(const BaseSubsystem *, void *args);

   /**
    * @brief Specification for dependencies of a threaded subsystem
    *
    */
   struct Spec {
      Spec(BaseSubsystem *subsys, BaseSubsystem** deps);
      /**
       * @brief a pointer to the subsystem to add
       *
       */
      BaseSubsystem *subsystem;

      /**
       * @brief null terminated array of threaded subsystem dependencies of this one
       *
       */
      BaseSubsystem **deps;
      Spec *next;
   };

   SubsystemManagerClass();
   virtual ~SubsystemManagerClass();

   /**
    * @brief add a subsystem to the subsystem manager
    *
    * @param spec the dependency specification of this subsystem
    *
    * In your constructor, use as:
    * static BaseSubsystem* deps[] = {&DependentSubsystemInstance, NULL};
    * static SubsystemManagerClass::Spec spec = {
    *    .subsystem = this,
    *    .deps = deps,
    * };
    * SubsystemManager.addSubsystem(&spec);
    */
   void addSubsystem(Spec *spec);
   Status setup();
   Status start();

   /**
    * @brief iterate over all subsystems the manager knows about
    *
    * @param fn function pointer to call
    * @param args arguments to call fn with
    */
   void iterateSubsystems(SubsystemFn fn, void *args);

private:
   Spec* specs;

   Spec* findSpecBySubsystem(BaseSubsystem *needle);
   void descendAndStartOrSetup(Spec *spec, BaseSubsystem::Status desiredState, int depth=0);
};

extern SubsystemManagerClass SubsystemManager;

