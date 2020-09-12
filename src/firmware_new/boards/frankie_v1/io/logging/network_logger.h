
#include "cmsis_os.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "shared/proto/tbots_log.nanopb.h"

/**
 * Network Logger Task
 *
 * Consumes msgs on the log queue, and sends it over the network with
 * the provided communication profile.
 *
 * @param communication_profile The profile to send the logs over
 */
void io_logging_network_logger_task(void* network_profile);

/**
 * Initialize network logger
 *
 * @param The log msg queue to consume logs
 */
void io_logging_network_logger_init(osMessageQueueId_t message_queue_id);
