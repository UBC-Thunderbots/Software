#include "ai_communicator.h"
#include "firmware_new/proto/control_fw.pb.h"
#include "lwip/api.h"
#include "lwip/inet.h"
#include "lwip/ip_addr.h"
#include "lwip/memp.h"
#include "lwip/opt.h"
#include "lwip/pbuf.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "main.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

uint8_t recv_buffer[robot_ack_size];
uint8_t send_buffer[control_msg_size];

int msg_count       = 0;
robot_ack ack       = robot_ack_init_zero;
control_msg control = control_msg_init_zero;
const char* multicast_ip;

unsigned udp_multicast_join_group(const char *multicast_ip, int multicast_port);
static void udp_multicast_thread(void *arg);

unsigned udp_multicast_join_group(const char *multicast_ip, int multicast_port)
{
    unsigned sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    struct sockaddr_in serv_addr;
    serv_addr.sin_family      = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(multicast_ip);
    serv_addr.sin_port        = htons(multicast_port);

    if (bind(sock, (struct sockaddr *)&serv_addr, (socklen_t)sizeof(serv_addr)) < 0)
        return -1;

    ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(multicast_ip);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);

    if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq)) < 0)
        return -1;
    return sock;
}

static void udp_multicast_thread(void *arg)
{
    unsigned socket = *((unsigned*)(arg));

    while (1)
    {
        int length = recv(socket, &recv_buffer, robot_ack_size, 0);

        // Create a stream that reads from the buffer
        pb_istream_t in_stream =
            pb_istream_from_buffer(recv_buffer, length);

        if (pb_decode(&in_stream, control_msg_fields, &control))
        {
            // update proto
            ack.msg_count = msg_count++;

            // serialize proto
            pb_ostream_t stream = pb_ostream_from_buffer(send_buffer, sizeof(send_buffer));
            pb_encode(&stream, robot_ack_fields, &ack);

            // package payload and send over udp
            send(socket, send_buffer, stream.bytes_written, 0);
        }
    }
}

void udp_multicast_init(const char *multicast_address, int multicast_port)
{
    // create a connection to the multicast group
    unsigned* socket = malloc(sizeof(int));
    *socket = udp_multicast_join_group(multicast_address, multicast_port);

    sys_thread_new("multicast_thread", udp_multicast_thread, socket, 1024,
                    (osPriority_t)osPriorityNormal);
}
