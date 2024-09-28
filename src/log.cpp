#include "log.h"
#include <USBCDC.h>

LogWriterClass LogWriter;

static void printPrefix(Print* _logOutput, int logLevel);

LogWriterClass::LogWriterClass() {
    name = "logwriter";
    queue = xQueueCreateStatic(QUEUE_SIZE,
            sizeof(uint8_t),
            queueStorage,
            &staticQueue);
    static BaseSubsystem* deps[] = {NULL};
    static SubsystemManagerClass::Spec spec(this, deps);
    SubsystemManager.addSubsystem(&spec);
    Log.setPrefix(printPrefix);
}

LogWriterClass::LogWriterClass::~LogWriterClass() {
}

BaseSubsystem::Status LogWriterClass::LogWriterClass::setup() {
    setStatus(BaseSubsystem::FAULT);
    if (queue == NULL) {
        error = "queue Null";
        goto out;
    }

    error = "none";
    Log.begin(LOG_LEVEL_VERBOSE, this);
    setStatus(BaseSubsystem::READY);

out:
    return getStatus();
}

int LogWriterClass::LogWriterClass::taskPriority() const {
    return tskIDLE_PRIORITY;
}

void LogWriterClass::LogWriterClass::taskFunction(void *parameter) {
    uint8_t c;
    size_t i = 0;

    while(1) {
        if (xQueueReceive(queue, &c, portMAX_DELAY) == pdPASS) {
            buf[i] = c;
            i++;
            if (c == '\n' || c == '\r' || i >= FLUSH_THRESHOLD - 1) {
                rwLock.Lock();
                for (auto j = 0; j < numPrinters; j++) {
                    auto print = printers[j];
                    if (print) {
                        print->write(buf, i);
                        print->flush();
                    }
                }
                rwLock.UnLock();
                i = 0;
            }
        }
    }
}

size_t LogWriterClass::LogWriterClass::write(uint8_t c) {
    size_t len = 0;

    if (xQueueSend(queue, (void *)&c, (TickType_t)0) == errQUEUE_FULL) {
        error = "overrun";
        goto out;
    }
    len = 1;
out:
    sendlock.UnLock();
    return len;
}

size_t LogWriterClass::write(const uint8_t *buffer, size_t size) {
    size_t len = 0;

    sendlock.Lock();
    for (auto i=0; i < size; i++) {
        if (xQueueSend(queue, (void *)&(buffer[i]), (TickType_t)0) == errQUEUE_FULL) {
            error = "overrun";
            goto out;
        }
        len++;
    }
out:
    sendlock.UnLock();
    return len;
}

size_t LogWriterClass::write(const char *str) {
    size_t len = 0;

    sendlock.Lock();
    if (str == NULL) {
        goto out;
    }
    for (auto i = 0; str[i]; i++) {
        if (xQueueSend(queue, (void *)&(str[i]), (TickType_t)0) == errQUEUE_FULL) {
            error = "overrun";
            goto out;
        }
        len++;
    }
out:
    sendlock.UnLock();
    return len;
}

void LogWriterClass::addPrinter(Print *print) {
    if (print == NULL) {
        return;
    }
    rwLock.Lock();
    if (numPrinters >= MAX_PRINTERS){
        goto out;
    }
    printers[numPrinters] = print;
    numPrinters++;

    out:
    rwLock.UnLock();
}

void LogWriterClass::addSerialPrinter() {
    addPrinter(&Serial);
}

// adapted from https://github.com/thijse/Arduino-Log/blob/master/examples/Log-advanced/Log-advanced.ino
static void printLogLevel(Print* _logOutput, int logLevel) {
    /// Show log description based on log level
    switch (logLevel) {
        default:
        case 0:_logOutput->print("SILENT " ); break;
        case 1:_logOutput->print("FATAL "  ); break;
        case 2:_logOutput->print("ERROR "  ); break;
        case 3:_logOutput->print("WARNING "); break;
        case 4:_logOutput->print("INFO "   ); break;
        case 5:_logOutput->print("TRACE "  ); break;
        case 6:_logOutput->print("VERBOSE "); break;
    }
}

static void printTimestamp(Print* _logOutput) {
  // Division constants
  static constexpr unsigned long MSECS_PER_SEC       = 1000;
  static constexpr unsigned long SECS_PER_MIN        = 60;
  static constexpr unsigned long SECS_PER_HOUR       = 3600;
  static constexpr unsigned long SECS_PER_DAY        = 86400;

  // Total time
  const unsigned long msecs               =  millis();
  const unsigned long secs                =  msecs / MSECS_PER_SEC;

  // Time in components
  const unsigned long MilliSeconds        =  msecs % MSECS_PER_SEC;
  const unsigned long Seconds             =  secs  % SECS_PER_MIN ;
  const unsigned long Minutes             = (secs  / SECS_PER_MIN) % SECS_PER_MIN;
  const unsigned long Hours               = (secs  % SECS_PER_DAY) / SECS_PER_HOUR;

  // Time as string
  char timestamp[20];
  snprintf(timestamp, sizeof(timestamp), "[%02d:%02d:%02d.%03d] ", Hours, Minutes, Seconds, MilliSeconds);
  _logOutput->print(timestamp);
}

static void printPrefix(Print* _logOutput, int logLevel) {
    printTimestamp(_logOutput);
    //printLogLevel (_logOutput, logLevel);
}
