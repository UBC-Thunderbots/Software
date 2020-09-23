
#include "cmsis_os.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "shared/proto/tbots_log.nanopb.h"

/**
 * Logger Task
 *
 * Consumes msgs on the log queue, and sends it over the network with
 * the provided communication profile.
 *
 * @param communication_profile The profile to send the logs over
 */
void io_logging_task(void* network_profile);

/**
 * Initialize logger. The logger will log to each sink in the array.
 *
 * @param log_sinks The log sinks to log to
 * @param num_sinks How many sinks are in the log_sinks array
 */
void io_logging_logger_init(const LogSink_t*[] log_sinks, size_t num_sinks);
