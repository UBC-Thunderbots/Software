#include "firmware/boards/robot_stm32h7/io/proto_multicast.h"

#include <assert.h>
#include <stdlib.h>

#include "firmware/app/logger/logger.h"
#include "firmware/boards/robot_stm32h7/io/proto_multicast_communication_profile.h"
#include "firmware/boards/robot_stm32h7/io/ublox_odinw262_communicator.h"
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
#include "main.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

// internal event to signal network interface configured
static osEventFlagsId_t networking_event;

// the timeout to recv a network packet
static int network_timeout_ms;

// flag mask for network interface configured event
// this flag is set on the networking_event to indicate "link up"
static uint32_t NETIF_CONFIGURED = 1 << 0;

void io_proto_multicast_init(int net_timeout_ms)
{
    networking_event   = osEventFlagsNew(NULL);
    network_timeout_ms = net_timeout_ms;
}

void io_proto_multicast_senderTask(void* communication_profile)
{
    ProtoMulticastCommunicationProfile_t* profile =
        (ProtoMulticastCommunicationProfile_t*)communication_profile;

    osEventFlagsWait(networking_event, NETIF_CONFIGURED, osFlagsWaitAny, osWaitForever);

    // Bind the socket to the multicast address and port we then use that
    // communication profile to join the specified multicast group.
    //
    // We use IP6_ADDR_ANY which will default to the ethernet interface on the STM32H7
    struct netconn* conn = netconn_new(NETCONN_UDP);

    netconn_bind(conn, IP6_ADDR_ANY,
                 io_proto_multicast_communication_profile_getPort(profile));

    netconn_join_leave_group(conn,
                             io_proto_multicast_communication_profile_getAddress(profile),
                             IP6_ADDR_ANY, NETCONN_JOIN);

    // this buffer is used to hold serialized proto
    uint8_t buffer[io_proto_multicast_communication_profile_getMaxProtoSize(profile)];

    // network buffer
    struct netbuf* tx_buf = netbuf_new();

    for (;;)
    {
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

        // block until this task receives a signal that the protobuf has been updated
        io_proto_multicast_communication_profile_blockUntilEvents(profile, PROTO_UPDATED);

        // serialize proto into buffer
        io_proto_multicast_communication_profile_acquireLock(profile);

        // we ignore the error returned by pb_encode, it is up to the receiver to handle
        // malformed proto, so we send the buffer regardless
        pb_encode(&stream,
                  io_proto_multicast_communication_profile_getProtoFields(profile),
                  io_proto_multicast_communication_profile_getProtoStruct(profile));

        io_proto_multicast_communication_profile_releaseLock(profile);

        // Max uint16 is 65535, which is significantly over the theoretical max
        // tranfser unit that can be configured at 9000 bytes. We can safely cast
        // stream.bytes_written to a (u16_t)
        netbuf_alloc(tx_buf, (u16_t)stream.bytes_written);

        // package payload and send over udp
        tx_buf->p->payload = buffer;
        netconn_sendto(conn, tx_buf,
                       io_proto_multicast_communication_profile_getAddress(profile),
                       io_proto_multicast_communication_profile_getPort(profile));
    }

    // we should never get here
    netbuf_delete(tx_buf);
    netconn_delete(conn);
}

void io_proto_multicast_listenerTask(void* communication_profile)
{
    ProtoMulticastCommunicationProfile_t* profile =
        (ProtoMulticastCommunicationProfile_t*)communication_profile;

    osEventFlagsWait(networking_event, NETIF_CONFIGURED, osFlagsWaitAny, osWaitForever);

    // Bind the socket to the multicast address and port we then use that
    // communication profile to join the specified multicast group.
    struct netconn* conn = netconn_new(NETCONN_UDP_IPV6);

    netconn_bind(conn, io_proto_multicast_communication_profile_getAddress(profile),
                 io_proto_multicast_communication_profile_getPort(profile));

    netconn_join_leave_group(conn,
                             io_proto_multicast_communication_profile_getAddress(profile),
                             IP6_ADDR_ANY, NETCONN_JOIN);

    // TODO: This is a hack to work with vision, this should really be user configurable
    netconn_set_recvtimeout(conn, 5);

    struct netbuf* rx_buf = NULL;
    err_t network_err;
    bool no_protobuf_err;

    for (;;)
    {
        network_err = netconn_recv(conn, &rx_buf);

        switch (network_err)
        {
            case ERR_TIMEOUT:
            {
                io_proto_multicast_communication_profile_notifyEvents(profile,
                                                                      RECEIVE_TIMEOUT);
                break;
            }
            case ERR_OK:
            {
                pb_istream_t in_stream = pb_istream_from_buffer(
                    (uint8_t*)rx_buf->p->payload, rx_buf->p->tot_len);

                io_proto_multicast_communication_profile_acquireLock(profile);

                // deserialize into buffer, nanopb err logic is inverted, false = error
                no_protobuf_err = pb_decode(
                    &in_stream,
                    io_proto_multicast_communication_profile_getProtoFields(profile),
                    io_proto_multicast_communication_profile_getProtoStruct(profile));

                io_proto_multicast_communication_profile_releaseLock(profile);

                if (no_protobuf_err)
                {
                    io_proto_multicast_communication_profile_notifyEvents(profile,
                                                                          RECEIVED_PROTO);
                }
                break;
            }
        }
        netbuf_delete(rx_buf);
    }

    // we should never get here
    netconn_delete(conn);
}

void io_proto_multicast_startNetworkingTask(void* unused)
{
    MX_LWIP_Init();
    /*io_ublox_odinw262_communicator_connectToWiFi();*/

    osEventFlagsSet(networking_event, NETIF_CONFIGURED);

    osThreadExit();
}
