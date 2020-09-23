
typedef struct LogSink LogSink_t;

/**
 * A Log Sink defines the behaviour for logs at different log levels
 *
 * @param log_debug A function pointer that handles debug level logs
 * @param log_info A function pointer that handles info level logs
 * @param log_warn A function pointer that handles warn level logs
 * @param log_fatal A function pointer that handles fatal level logs
 *
 * @returns LogSink_t The Log Sink
 */
LogSink_t* io_logging_log_sink_create(void (*log_debug)(const char*),
                                      void (*log_info)(const char*),
                                      void (*log_warn)(const char*),
                                      void (*log_fatal)(const char*));
