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
#include "shared/constants.h"

typedef struct ProtoMulticastConnectionProfile
{
    // the multicast group to join and the port to bind to
    ip_addr_t* multicast_address;
    uint16_t port;

    // nanopb generates an array containing the order of the
    // fields in the protobuf, that the pb_encode/pb_decode
    // functions need to understand the connects of the msg
    const pb_field_t[] message_fields;

    // the maximum known size of the protobuf. if the
    // maaximum size is undeterminable (i.e repated fields)
    // this value should be set to MAXIMUM_TRANSFER_UNIT of the network packet
    uint16_t message_max_size;

} ProtoMulticastConnectionProfile_t;


typedef struct ProtoMulticastSenderProfile
{
    ProtoMulticastConnectionProfile_t connection_profile;

    const void* protobuf_struct;

    uint16_t sending_rate_hertz;

} ProtoMulticastSenderProfile_t;

typedef struct ProtoMulticastListenerProfile
{
    ProtoMulticastConnectionProfile_t connection_profile;

    const void* protobuf_struct;

    uint16_t timeout_period_seconds;
    void (*timeout_callback)(void):
    
} ProtoMulticastListenerProfile_t;

ProtoMulticastCommunicationProfile_t* io_proto_multicast_profile_create(
    const char* multicast_address, uint16_t port, const void* protobuf_struct,
    const pb_field_t[] message_fields, uint16_t message_max_size)
{
    ProtoMulticastCommunicationProfile_t* connection_profile =
        (ProtoMulticastCommunicationProfile_t*)malloc(
            sizeof(ProtoMulticastCommunicationProfile_t));

    connection_profile->multicast_address = (ip_addr_t*)malloc(sizeof(ip_addr_t));
    ip6addr_aton(multicast_address, multicast_address);

    connection_profile->port             = port;
    connection_profile->protobuf_struct  = protobuf_struct;
    connection_profile->message_fields   = message_fields;
    connection_profile->message_max_size = message_max_size;
}

ProtoMulticastListenerProfile_t* io_proto_multicast_listener_profile_create(
    const char* multicast_address, uint16_t port, const void* protobuf_struct,
    const pb_field_t[] message_fields, uint16_t message_max_size,
    uint16_t timeout_millisecoonds, void (*timeout_callback)(void))
{


}

void io_proto_multicast_sender_profile_destroy(
    ProtoMulticastCommunicationProfile_t* connection_profile)
{
    free(connection_profile->multicast_address);
    free(connection_profile);
}

void io_proto_multicast_listener_profile_destroy(
    ProtoMulticastCommunicationProfile_t* connection_profile)
{
    free(connection_profile->multicast_address);
    free(connection_profile);
}
