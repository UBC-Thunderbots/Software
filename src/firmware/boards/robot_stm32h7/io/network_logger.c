#include "firmware/boards/robot_stm32h7/io/network_logger.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "firmware/boards/robot_stm32h7/io/proto_multicast_communication_profile.h"
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
    TbotsProto_RobotLog dequeued_robot_log_proto;

    ProtoMulticastCommunicationProfile_t* profile =
        (ProtoMulticastCommunicationProfile_t*)communication_profile;

    osStatus_t status;

    for (;;)
    {
        status = osMessageQueueGet(log_message_queue_id_, &dequeued_robot_log_proto, NULL,
                                   osWaitForever);
        if (status == osOK)
        {
            io_proto_multicast_communication_profile_acquireLock(profile);
            memcpy(io_proto_multicast_communication_profile_getProtoStruct(profile),
                   &dequeued_robot_log_proto, sizeof(TbotsProto_RobotLog));
            io_proto_multicast_communication_profile_releaseLock(profile);
            io_proto_multicast_communication_profile_notifyEvents(profile, PROTO_UPDATED);
        }
    }
}

void io_network_logger_init(osMessageQueueId_t message_queue_id)
{
    log_message_queue_id_ = message_queue_id;
}

void io_network_logger_handleRobotLog(TbotsProto_RobotLog robot_log)
{
    // NOTE: we pass in a ptr to the robot_log on the stack, normally this can be
    // catastrophic, but osMessageQueuePut guarantees that the msg is copied, and
    // the memory at this pointer location does _not_ need to be preserved after
    // calling this function.
    osMessageQueuePut(log_message_queue_id_, &robot_log, 0, 0);
}
