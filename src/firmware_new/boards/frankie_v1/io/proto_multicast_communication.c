#include "firmware_new/boards/frankie_v1/io/proto_multicast_communication.h"

#include <stdlib.h>

#include "lwip.h"
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

static osEventFlagsId_t networking_event;
static uint32_t NETIF_CONFIGURED = 1 << 0;

/**
 * ProtoMulticastCommunicationProfile_t contains the common information
 * between to join a multicast group and then send or receive protobuf.
 */
typedef struct ProtoMulticastCommunicationProfile
{
    const char* profile_name;

    // multicast group info: the address and the port to bind to and
    // join to send/recv proto
    ip_addr_t* multicast_address;
    uint16_t port;

    // protobuf info: the pb_field_t array fully defines the protobuf_struct,
    // so we use a void pointer to be generic to any protobuf message.
    void* protobuf_struct;
    const pb_field_t* message_fields;
    uint16_t message_max_size;

    // communication_event: these events will be used to control when the networking
    // tasks run. The networking tasks will also signal certain events
    osEventFlagsId_t communication_event;
    osEventFlagsId_t netif_event;

    // mutex to protect the protobuf struct
    osMutexId_t profile_mutex;

} ProtoMulticastCommunicationProfile_t;

ProtoMulticastCommunicationProfile_t* io_proto_multicast_communication_profile_create(
    const char* profile_name, const char* multicast_address, uint16_t port,
    void* protobuf_struct, const pb_field_t* message_fields, uint16_t message_max_size)
{
    ProtoMulticastCommunicationProfile_t* profile =
        (ProtoMulticastCommunicationProfile_t*)malloc(
            sizeof(ProtoMulticastCommunicationProfile_t));

    const osMutexAttr_t mutex_attr = {profile_name, osMutexPrioInherit, NULL, 0U};

    profile->profile_name        = profile_name;
    profile->port                = port;
    profile->message_fields      = message_fields;
    profile->message_max_size    = message_max_size;
    profile->protobuf_struct     = protobuf_struct;
    profile->profile_mutex       = osMutexNew(&mutex_attr);
    profile->communication_event = osEventFlagsNew(NULL);
    profile->multicast_address   = (ip_addr_t*)malloc(sizeof(ip_addr_t));
    ip6addr_aton(multicast_address, profile->multicast_address);
}

void io_proto_multicast_sender_Task(void* arg)
{
    ProtoMulticastCommunicationProfile_t* comm_profile =
        (ProtoMulticastCommunicationProfile_t*)arg;

    osEventFlagsWait(networking_event, NETIF_CONFIGURED, osFlagsWaitAny, osWaitForever);

    // Bind the socket to the multicast address and port we then use that
    // communication profile to join the specified multicast group.
    //
    // We use IP6_ADDR_ANY which will default to the ethernet interface on the STM32H7
    struct netconn* conn = netconn_new(NETCONN_UDP_IPV6);
    netconn_bind(conn, IP6_ADDR_ANY, comm_profile->port);
    netconn_join_leave_group(conn, comm_profile->multicast_address, NULL, NETCONN_JOIN);

    // this buffer is used to hold serialized proto
    uint8_t buffer[comm_profile->message_max_size];

    // network buffer
    struct netbuf* tx_buf = NULL;

    for (;;)
    {
        osEventFlagsWait(comm_profile, UPDATED_PROTO, osFlagsWaitAny, osWaitForever);
        tx_buf = netbuf_new();

        netbuf_alloc(tx_buf, comm_profile->message_max_size);

        // serialize proto
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        pb_encode(&stream, comm_profile->message_fields, comm_profile->protobuf_struct);

        // package payload and send over udp
        tx_buf->p->payload = buffer;
        netconn_sendto(conn, tx_buf, comm_profile->multicast_address, comm_profile->port);

        osThreadYield();
        netbuf_delete(tx_buf);
    }
}

void io_proto_multicast_listener_Task(void* arg)
{
    ProtoMulticastCommunicationProfile_t* comm_profile =
        (ProtoMulticastCommunicationProfile_t*)arg;

    osEventFlagsWait(networking_event, NETIF_CONFIGURED, osFlagsWaitAny, osWaitForever);

    // Bind the socket to the multicast address and port we then use that comm_profile
    // profile to join the specified multicast group.
    struct netconn* conn = netconn_new(NETCONN_UDP_IPV6);
    netconn_bind(conn, comm_profile->multicast_address, comm_profile->port);
    netconn_join_leave_group(conn, comm_profile->multicast_address, NULL, NETCONN_JOIN);

    // network buffer
    struct netbuf* rx_buf = NULL;

    // TODO proper err handling, for now all errs are ignored
    // https://github.com/UBC-Thunderbots/Software/issues/1190
    err_t err;

    for (;;)
    {
        err = netconn_recv(conn, &rx_buf);

        if (err == ERR_OK)
        {
            // Create a stream that reads from the buffer
            pb_istream_t in_stream =
                pb_istream_from_buffer((uint8_t*)rx_buf->p->payload, rx_buf->p->tot_len);

            if (pb_decode(&in_stream, comm_profile->message_fields,
                          comm_profile->protobuf_struct))
            {
                // TODO HANDLE MALFORMED PROTO
            }
        }

        netbuf_delete(rx_buf);
    }
}

void io_proto_multicast_startNetworkingTask(void* arg)
{
    networking_event = osEventFlagsNew(NULL);
    MX_LWIP_Init();
    osEventFlagsSet(networking_event, NETIF_CONFIGURED);
    osThreadExit();
}

void io_proto_multicast_communication_profile_destroy(
    ProtoMulticastCommunicationProfile_t* profile)
{
    osMutexDelete(profile->profile_mutex);
    free(profile->multicast_address);
    free(profile);
}
