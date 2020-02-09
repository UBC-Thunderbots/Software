#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "main.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "ai_communicator.h"


// private functions
int multicast_bind(int sock, uint16_t port);
int join_group(int sock, const char* join_ip, const char* local_ip);
void multicast_start(const char* multicast_address);

int multicast_bind(int sock, uint16_t port)
{
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = IPADDR_ANY;
    serv_addr.sin_port = htons(port);

    if (bind(sock, (struct sockaddr *) &serv_addr, (socklen_t)sizeof(serv_addr)) < 0)
        return -1;
    return 0;
}

int join_group(int sock, const char* join_ip, const char* local_ip)
{
    ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(join_ip);
    mreq.imr_interface.s_addr = inet_addr(local_ip);

    if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq)) < 0)
        return -1;
    return 0;
}

void multicast_start(const char* multicast_address)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    multicast_bind(sock, 5000);
    join_group(sock, multicast_address, IP_ADDR_ANY);
}


static void ai_communicator_thread(void *arg)
{
    LWIP_UNUSED_ARG(arg);

    struct netconn *conn;
    struct netbuf *buf, *tx_buf;
    err_t err;

    conn = netconn_new(NETCONN_UDP);
    netconn_bind(conn, IP_ADDR_ANY, 42069);

    LWIP_ERROR("ai_communicator: cannot connect", (conn != NULL), return;);

    while (1) {

        err = netconn_recv(conn, &buf);

        if (err == ERR_OK) {

            tx_buf = netbuf_new();
            netbuf_alloc(tx_buf, robot_ack_size);

            // Create a stream that reads from the buffer
            pb_istream_t in_stream = pb_istream_from_buffer(
                    (uint8_t * )buf->p->payload, buf->p->tot_len);

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

void ai_communicator_init(void)
{
    multicast_start("10.10.10.10");
    sys_thread_new("ai_communicator", ai_communicator_thread, NULL,
                            (configMINIMAL_STACK_SIZE*2), osPriorityAboveNormal);
}
