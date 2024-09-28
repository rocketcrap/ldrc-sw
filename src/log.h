#pragma once

#include <subsystem.h>
#include <Print.h>
#include <ArduinoLog.h>


/**
 * @brief A logger based upon https://github.com/thijse/Arduino-Log
 *
 * This implementaiton is thread-safe and is ready to begin logging as soon as the logger is constructed. Add printers with AddPrinter, a convenience function of addSerial() is provided
 *
 * It will not start outputting until the thread is started.
 *
 */
class LogWriterClass : public ThreadedSubsystem, public Print {
    public:
        LogWriterClass();
        virtual ~LogWriterClass();
        BaseSubsystem::Status setup();

        /**
         * @brief add a Printer to the LogWriter
         *
         * Ensure you call this prior to run on LogWriter being called to ensure you get all messages.
         *
         * @param print the printer to print to
         */
        void addPrinter(Print *print);

        /**
         * @brief add the hardware serial port as a printer
         *
         */
        void addSerialPrinter();

        /**
         * @brief holder for internal errors
         *
         */
        const char *error;

        virtual size_t write(uint8_t c);
        virtual size_t write(const uint8_t *buffer, size_t size);
        virtual size_t write(const char *str);

    protected:
        virtual int taskPriority() const;
        virtual void taskFunction(void *parameter);

    private:
	    static constexpr size_t QUEUE_SIZE = 1024*2;
        static constexpr size_t FLUSH_THRESHOLD = 81;
        static constexpr size_t MAX_PRINTERS = 8;

        StaticQueue_t staticQueue;
        QueueHandle_t queue;
        uint8_t queueStorage[QUEUE_SIZE * sizeof(uint8_t)];

        uint8_t buf[FLUSH_THRESHOLD];

        Print* printers[MAX_PRINTERS];
        size_t numPrinters;

        ReadWriteLock sendlock;
};

extern LogWriterClass LogWriter;