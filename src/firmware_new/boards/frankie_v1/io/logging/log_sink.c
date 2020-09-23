#include "log_sink.h"

typedef struct LogSink
{
    void (*log_debug)(const char*);
    void (*log_info)(const char*);
    void (*log_warn)(const char*);
    void (*log_fatal)(const char*);
} LogSink_t;

LogSink_t* io_logging_log_sink_create(void (*log_debug)(const char*),
                                      void (*log_info)(const char*),
                                      void (*log_warn)(const char*),
                                      void (*log_fatal)(const char*))
{
    LogSink_t* log_sink = (LogSink_t*)malloc(sizeof(LogSink_t));

    log_sink->log_debug = log_debug;
    log_sink->log_info = log_info;
    log_sink->log_warn = log_warn;
    log_sink->log_fatal = log_fatal;
}
