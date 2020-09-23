#include "cmsis_os.h"
#include "logger.h"
#include "pb.h"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "shared/proto/tbots_log.nanopb.h"

// the id of the queue that contains tbots log protos
osMessageQueueId_t log_message_queue_id;

void io_logging_network_logger_task(void* communication_profile)
{
    // holds the msg that was just dequeued
    void* ptr_to_tbots_log_proto = NULL;

    /* Infinite loop */
    while (osMessageQueueGet(log_message_queue_id, ptr_to_tbots_log_proto, NULL, 0))
    {
    }
}

void io_logging_network_logger_init(osMessageQueueId_t message_queue_id)
{
    log_message_queue_id = message_queue_id;
}
