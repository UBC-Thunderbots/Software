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

typedef struct ProtoMulticastConnectionProfile
{
    ip_addr_t* multicast_address;
    uint16_t port;
    const pb_field_t* message_fields;
    uint16_t message_max_size;

} ProtoMulticastConnectionProfile_t;

typedef struct ProtoMulticastSenderProfile
{
    ProtoMulticastConnectionProfile_t* connection_profile;
    const void* protobuf_struct;
    uint16_t sending_rate_hertz;

} ProtoMulticastSenderProfile_t;

typedef struct ProtoMulticastListenerProfile
{
    ProtoMulticastConnectionProfile_t* connection_profile;
    const void* protobuf_struct;
    uint16_t timeout_milliseconds;
    void (*timeout_callback)(void);

} ProtoMulticastListenerProfile_t;

ProtoMulticastConnectionProfile_t* io_proto_multicast_connection_profile_create(
    const char* multicast_address, uint16_t port, const void* protobuf_struct,
    const pb_field_t* message_fields, uint16_t message_max_size)
{
    ProtoMulticastConnectionProfile_t* connection_profile =
        (ProtoMulticastConnectionProfile_t*)malloc(
            sizeof(ProtoMulticastConnectionProfile_t));

    connection_profile->multicast_address = (ip_addr_t*)malloc(sizeof(ip_addr_t));
    ip6addr_aton(multicast_address, multicast_address);

    connection_profile->port             = port;
    connection_profile->message_fields   = message_fields;
    connection_profile->message_max_size = message_max_size;
    return connection_profile;
}

ProtoMulticastListenerProfile_t* io_proto_multicast_listener_profile_create(
    const char* multicast_address, uint16_t port, const void* protobuf_struct,
    const pb_field_t* message_fields, uint16_t message_max_size,
    uint16_t timeout_milliseconds, void (*timeout_callback)(void))
{
    ProtoMulticastListenerProfile_t* listener_profile =
        (ProtoMulticastListenerProfile_t*)malloc(sizeof(ProtoMulticastListenerProfile_t));

    listener_profile->connection_profile = io_proto_multicast_connection_profile_create(
        multicast_address, port, protobuf_struct, message_fields, message_max_size);

    listener_profile->protobuf_struct      = protobuf_struct;
    listener_profile->timeout_milliseconds = timeout_milliseconds;
    listener_profile->timeout_callback     = timeout_callback;
    return listener_profile;
}

ProtoMulticastSenderProfile_t* io_proto_multicast_sender_profile_create(
    const char* multicast_address, uint16_t port, const void* protobuf_struct,
    const pb_field_t* message_fields, uint16_t message_max_size,
    uint16_t sending_rate_hertz)
{
    ProtoMulticastSenderProfile_t* sender_profile =
        (ProtoMulticastSenderProfile_t*)malloc(sizeof(ProtoMulticastSenderProfile_t));

    sender_profile->connection_profile = io_proto_multicast_connection_profile_create(
        multicast_address, port, protobuf_struct, message_fields, message_max_size);

    sender_profile->protobuf_struct    = protobuf_struct;
    sender_profile->sending_rate_hertz = sending_rate_hertz;
    return sender_profile;
}

void io_proto_multicast_listener_profile_destroy(
    ProtoMulticastListenerProfile_t* listener_profile)
{
    free(listener_profile->connection_profile->multicast_address);
    free(listener_profile->connection_profile);
    free(listener_profile);
}

void io_proto_multicast_sender_profile_destroy(
    ProtoMulticastSenderProfile_t* sender_profile)
{
    free(sender_profile->connection_profile->multicast_address);
    free(sender_profile->connection_profile);
    free(sender_profile);
}

void io_proto_multicast_sender_Task(void* sender_profile)
{
    ProtoMulticastSenderProfile_t* proto_multicast_sender =
        (ProtoMulticastSenderProfile_t*)sender_profile;
    for (;;)
    {
    }
}

void io_proto_multicast_listener_Task(void* listener_profile)
{
    ProtoMulticastListenerProfile_t* proto_multicast_listener =
        (ProtoMulticastListenerProfile_t*)listener_profile;
    for (;;)
    {
    }
}
