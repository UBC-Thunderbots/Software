#include "firmware_new/boards/frankie_v1/io/proto_multicast_communication.h"

#include <stdlib.h>

#include "lwip/api.h"
#include "lwip/inet.h"
#include "lwip/ip_addr.h"
#include "lwip/memp.h"
#include "lwip/opt.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

/**
 * ProtoMulticastConnectionProfile contains the common information
 * between to join a multicast group and then send or receive protobuf.
 */
typedef struct ProtoMulticastConnectionProfile
{
    // multicast group info:
    // the address and the port to bind to and join to send/recv proto
    ip_addr_t* multicast_address;
    uint16_t port;

    // protobuf info:
    // the pb_field_t array fully defines the protobuf_struct, so we use
    // a void pointer to be generic to any protobuf message.
    const pb_field_t* message_fields;
    uint16_t message_max_size;
    const void* protobuf_struct;

} ProtoMulticastConnectionProfile_t;

typedef struct ProtoMulticastSenderProfile
{
    // common connection profile
    ProtoMulticastConnectionProfile_t* connection_profile;

    // the rate the sending task should send the proto at
    uint16_t sending_rate_hertz;

} ProtoMulticastSenderProfile_t;

typedef struct ProtoMulticastListenerProfile
{
    // common connection profile
    ProtoMulticastConnectionProfile_t* connection_profile;

    // the timeout when msgs have been stopped being received
    uint16_t listening_timeout_milliseconds;
    void (*listening_timeout_callback)(void);

} ProtoMulticastListenerProfile_t;

ProtoMulticastConnectionProfile_t* io_proto_multicast_profile_create(
    const char* multicast_address, uint16_t port, const void* protobuf_struct,
    const pb_field_t* message_fields, uint16_t message_max_size)
{
    ProtoMulticastConnectionProfile_t* profile =
        (ProtoMulticastConnectionProfile_t*)malloc(
            sizeof(ProtoMulticastConnectionProfile_t));

    profile->multicast_address = (ip_addr_t*)malloc(sizeof(ip_addr_t));

    ip6addr_aton(multicast_address, multicast_address);
    profile->port             = port;
    profile->message_fields   = message_fields;
    profile->message_max_size = message_max_size;
    profile->protobuf_struct  = protobuf_struct;
}

ProtoMulticastListenerProfile_t* io_proto_multicast_listener_profile_create(
    const char* multicast_address, uint16_t port, const void* protobuf_struct,
    const pb_field_t* message_fields, uint16_t message_max_size,
    uint16_t listening_timeout_milliseconds, void (*listening_timeout_callback)(void))
{
    ProtoMulticastConnectionProfile_t* connection = io_proto_multicast_profile_create(
        multicast_address, port, protobuf_struct, message_fields, message_max_size);

    ProtoMulticastListenerProfile_t* listener_profile =
        (ProtoMulticastListenerProfile_t*)malloc(sizeof(ProtoMulticastListenerProfile_t));

    listener_profile->connection_profile             = connection;
    listener_profile->listening_timeout_milliseconds = listening_timeout_milliseconds;
    listener_profile->listening_timeout_callback     = listening_timeout_callback;

    return listener_profile;
}

ProtoMulticastSenderProfile_t* io_proto_multicast_sender_profile_create(
    const char* multicast_address, uint16_t port, const void* protobuf_struct,
    const pb_field_t* message_fields, uint16_t message_max_size,
    uint16_t sending_rate_hertz)
{
    ProtoMulticastConnectionProfile_t* connection = io_proto_multicast_profile_create(
        multicast_address, port, protobuf_struct, message_fields, message_max_size);

    ProtoMulticastSenderProfile_t* sender_profile =
        (ProtoMulticastSenderProfile_t*)malloc(sizeof(ProtoMulticastSenderProfile_t));

    sender_profile->connection_profile = connection;
    sender_profile->sending_rate_hertz = sending_rate_hertz;

    return sender_profile;
}

void io_proto_multicast_listener_profile_destroy(ProtoMulticastListenerProfile_t* profile)
{
    free(profile->connection_profile->multicast_address);
    free(profile->connection_profile);
    free(profile);
}

void io_proto_multicast_sender_profile_destroy(ProtoMulticastSenderProfile_t* profile)
{
    free(profile->connection_profile->multicast_address);
    free(profile->connection_profile);
    free(profile);
}

void io_proto_multicast_sender_Task(void* arg)
{
    ProtoMulticastSenderProfile_t* sender_profile = (ProtoMulticastSenderProfile_t*)arg;

    for (;;)
    {
        // TODO
        osDelay(100);
    }
}

void io_proto_multicast_listener_Task(void* arg)
{
    ProtoMulticastListenerProfile_t* listener_profile =
        (ProtoMulticastListenerProfile_t*)arg;

    for (;;)
    {
        // TODO
        osDelay(100);
    }
}
