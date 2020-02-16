#include "ai_communicator.h"
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


uint8_t buffer[robot_ack_size];
int msg_count       = 0;
robot_ack ack       = robot_ack_init_zero;
control_msg control = control_msg_init_zero;

struct netconn *udp_multicast_join_group(const char *multicast_ip, int multicast_port);
static void udp_multicast_thread(void *arg);

struct netconn *udp_multicast_join_group(const char *multicast_ip, int multicast_port)
{
    // create a new UDP connection on the heap
    struct netconn *conn = netconn_new(NETCONN_UDP);

    // create multicast address and port from arguments
    // htons swaps bytes to correct the endianness of the port,
    // due to netconns requirements
    ip_addr_t multicast_address;
    ip4addr_aton(multicast_ip, &multicast_address);

    // bind the socket to the multicast address and port
    // we then use that connection profile to join the specified
    // multicast group
    err_t test = netconn_bind(conn, &multicast_address, multicast_port);

    err_t err =
        netconn_join_leave_group(conn, &multicast_address, NULL, NETCONN_JOIN);

    if (test != ERR_OK)
    {
        return NULL;
    }

    if (err == ERR_OK)
    {
        return conn;
    }

    return NULL;
}

static void udp_multicast_thread(void *arg)
{
    struct netconn *conn = (struct netconn *)arg;
    struct netbuf *buf, *tx_buf;
    err_t err;

    while (1)
    {
        err = netconn_recv(conn, &buf);

        if (err == ERR_OK)
        {
            tx_buf = netbuf_new();
            netbuf_alloc(tx_buf, robot_ack_size);

            // Create a stream that reads from the buffer
            pb_istream_t in_stream =
                pb_istream_from_buffer((uint8_t *)buf->p->payload, buf->p->tot_len);

            if (pb_decode(&in_stream, control_msg_fields, &control))
            {
                // update proto
                ack.msg_count = msg_count++;

                // serialize proto
                pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
                pb_encode(&stream, robot_ack_fields, &ack);

                // package payload and send over udp
                tx_buf->p->payload = buffer;
                netconn_sendto(conn, tx_buf, (const ip_addr_t *)&(buf->addr), buf->port);
            }
            netbuf_delete(tx_buf);
        }
        netbuf_delete(buf);
    }
}

void udp_multicast_init(const char *multicast_address, int multicast_port)
{
    // create a connection to the multicast group
    struct netconn *conn = udp_multicast_join_group(multicast_address, multicast_port);

    sys_thread_new("multicast_thread", udp_multicast_thread, conn, 1024, (osPriority_t)osPriorityNormal);
}
