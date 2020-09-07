#include "network_logger.h"

#include "cmsis_os.h"
#include "firmware_new/boards/frankie_v1/io/networking/proto_multicast_communication_profile.h"
#include "pb.h"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "shared/proto/tbots_log.nanopb.h"

// the id of the queue that contains tbots log protos
osMessageQueueId_t log_message_queue_id;

void io_logging_network_logger_task(void* communication_profile)
{
    // the profile to send logs over
    ProtoMulticastCommunicationProfile_t* comm_profile =
        (ProtoMulticastCommunicationProfile_t*)communication_profile;

    // holds the msg that was just dequeued
    void* ptr_to_tbots_log_proto;

    /* Infinite loop */
    while (osMessageQueueGet(log_message_queue_id, ptr_to_tbots_log_proto, NULL, 0))
    {
        io_proto_multicast_communication_profile_acquireLock(comm_profile);

        TbotsProto_TbotsLog* log_msg_to_send =
            (TbotsProto_TbotsLog*)io_proto_multicast_communication_profile_getProtoStruct(
                comm_profile);

        // copy the data to send into the proto tracked in the profile
        *log_msg_to_send = *(TbotsProto_TbotsLog*)ptr_to_tbots_log_proto;

        io_proto_multicast_communication_profile_releaseLock(comm_profile);
        io_proto_multicast_communication_profile_notifyEvents(comm_profile,
                                                              PROTO_UPDATED);
    }
}

void io_logging_network_logger_init(osMessageQueueId_t message_queue_id)
{
    log_message_queue_id = message_queue_id;
}
