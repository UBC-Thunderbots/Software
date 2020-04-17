#include "udp_multicast.h"

#include "firmware_new/proto/control_fw.pb.h"
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

// TODO the messages are global for now, will need to be moved
// when this file is properly integrated. This file only serves
// as a reference implementation of reliable multicast with IPv6
RobotAck ack       = RobotAck_init_zero;
ControlMsg control = ControlMsg_init_zero;

/*
 * Thread that creates a send and recv socket, joins the specified
 * multicast group and start listening for packets. Increments a counter
 * on every multicast packet and sends a unicast packet back to the sender.
 * @param arg A void ptr to a multicast_config_t
 * @returns nothing *runs forever*
 *
 */
static void blocking_udp_multicast_loop(void *arg);

struct multicast_config
{
    ip_addr_t multicast_address;
    unsigned multicast_port;
    unsigned send_port;
} typedef multicast_config_t;

static void blocking_udp_multicast_loop(void *arg)
{
    multicast_config_t *config = (multicast_config_t *)arg;
    struct netbuf *rx_buf      = NULL;
    struct netbuf *tx_buf      = NULL;
    int msg_count              = 0;

    // TODO proper err handling, for now all errs are ignored
    // https://github.com/UBC-Thunderbots/Software/issues/1190
    err_t err;

    // create two new UDP connections on the heap
    // NOTE: we need two seperate UDP sockets as we will be
    // receiving multicast but sending unicast packets to avoid
    // network congestion (peer robots don't need to know the status
    // of other robots)
    struct netconn *recvconn = netconn_new(NETCONN_UDP_IPV6);
    struct netconn *sendconn = netconn_new(NETCONN_UDP_IPV6);

    // Bind the socket to the multicast address and port
    // we then use that connection profile to join the specified
    // multicast group.
    //
    // For the send socket, we use IP6_ADDR_ANY
    // which will default to the ethernet interface on the STM32H7
    //
    // This will remain the same regardless of communicating over ethernet
    // or WiFi because both of those media use the ETH interface
    netconn_bind(recvconn, &config->multicast_address, config->multicast_port);
    netconn_bind(sendconn, IP6_ADDR_ANY, config->send_port);
    netconn_join_leave_group(recvconn, &config->multicast_address, NULL, NETCONN_JOIN);

    // this buffer is used to hold serialized proto
    uint8_t buffer[RobotAck_size];

    while (1)
    {
        err = netconn_recv(recvconn, &rx_buf);

        if (err == ERR_OK)
        {
            tx_buf = netbuf_new();
            netbuf_alloc(tx_buf, RobotAck_size);

            // Create a stream that reads from the buffer
            pb_istream_t in_stream =
                pb_istream_from_buffer((uint8_t *)rx_buf->p->payload, rx_buf->p->tot_len);

            if (pb_decode(&in_stream, ControlMsg_fields, &control))
            {
                // update proto
                ack.msg_count = msg_count++;

                // serialize proto
                pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
                pb_encode(&stream, RobotAck_fields, &ack);

                // package payload and send over udp
                tx_buf->p->payload = buffer;
                netconn_sendto(sendconn, tx_buf, (const ip_addr_t *)&(rx_buf->addr),
                               config->send_port);
            }
            netbuf_delete(tx_buf);
        }
        netbuf_delete(rx_buf);
    }
}

void udp_multicast_init(const char *multicast_address, unsigned multicast_port,
                        unsigned send_port)
{
    multicast_config_t *config = malloc(sizeof(multicast_config_t));

    *config = (multicast_config_t){
        .multicast_port = multicast_port,
        .send_port      = send_port,
    };

    ip6addr_aton(multicast_address, &config->multicast_address);

    sys_thread_new("multicast_thread", blocking_udp_multicast_loop, config, 1024,
                   (osPriority_t)osPriorityNormal);
}
