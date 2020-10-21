#include "firmware_new/boards/frankie_v1/io/network_logger.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "firmware_new/boards/frankie_v1/io/proto_multicast_communication_profile.h"
#include "main.h"
#include "pb.h"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "shared/proto/robot_log_msg.nanopb.h"
#include "task.h"

// the id of the queue that contains tbots log protos
static osMessageQueueId_t log_message_queue_id_;

void io_network_logger_task(void* communication_profile)
{
    // holds the msg that was just dequeued
    void* ptr_to_tbots_log_proto = NULL;

    ProtoMulticastCommunicationProfile_t* profile =
        (ProtoMulticastCommunicationProfile_t*)communication_profile;

    /* Infinite loop */
    while (osMessageQueueGet(log_message_queue_id_, ptr_to_tbots_log_proto, NULL,
                             osWaitForever))
    {
        if (ptr_to_tbots_log_proto != NULL)
        {
            io_proto_multicast_communication_profile_acquireLock(profile);
            memcpy(io_proto_multicast_communication_profile_getProtoStruct(profile),
                   ptr_to_tbots_log_proto, sizeof(TbotsProto_RobotLog));
            io_proto_multicast_communication_profile_releaseLock(profile);
            io_proto_multicast_communication_profile_notifyEvents(profile, PROTO_UPDATED);
        }
    }
}

void io_network_logger_init(osMessageQueueId_t message_queue_id)
{
    log_message_queue_id_ = message_queue_id;
}

void io_network_logger_handle_robot_log_msg(TbotsProto_RobotLog log_msg)
{
    // NOTE: we pass in a ptr to the log_msg on the stack, normally this can lead
    // to disasters, but osMessageQueuePut gaurantees that the msg is copied, and
    // the memory at this pointer location does _not_ need to be preserved after
    // calling this function.
    osMessageQueuePut(log_message_queue_id_, &log_msg, 0, 0);
}
