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

// TODO remove
int msg_count = 0;


// these buffers are used to serialize and deserialize protobuf
uint8_t recv_buffer[robot_ack_size];
uint8_t send_buffer[control_msg_size];

// global proto msgs updated by the udp_multicast_thread
robot_ack ack       = robot_ack_init_zero;
control_msg control = control_msg_init_zero;

struct multicst_config
{
    struct sockaddr_in serv_addr;  // the address of the server
    ip_mreq mreq;                  // multicast request socket option
} typedef multicast_config_t;

static void udp_multicast_thread(void *arg);

/*
 * Joins the multicast group on the given port from the ethernet interface
 * If either socket call was unsuccessfull, the array will have a -1 instead
 * of the socket descriptor
 *
 * @param multicast_ip The IP to join
 * @param multicast_port The port to bind to
 *
 */
static void udp_multicast_thread(void *arg)
{
    multicast_config_t config = *(multicast_config_t *)arg;

    unsigned sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    bind(sock, (struct sockaddr *)&config.serv_addr, (socklen_t)sizeof(config.serv_addr));
    setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&config.mreq,
               sizeof(config.mreq));

    while (1)
    {
        int length = recv(sock, &recv_buffer, robot_ack_size, 0);

        // Create a stream that reads from the buffer
        pb_istream_t in_stream = pb_istream_from_buffer(recv_buffer, length);

        if (pb_decode(&in_stream, control_msg_fields, &control))
        {
            // update proto
            ack.msg_count = msg_count++;

            // serialize proto
            pb_ostream_t stream =
                pb_ostream_from_buffer(send_buffer, sizeof(send_buffer));
            pb_encode(&stream, robot_ack_fields, &ack);

            // package payload and send over udp
            sendto(sock, send_buffer, stream.bytes_written, 0,
                   (struct sockaddr *)&config.serv_addr,
                   (socklen_t)sizeof(config.serv_addr));
        }
    }
}

void udp_multicast_init(const char *multicast_address, int multicast_port)
{
    multicast_config_t *config = malloc(sizeof(multicast_config_t));

    *config = (multicast_config_t){
        .serv_addr =
            {
                .sin_family      = AF_INET,
                .sin_addr.s_addr = inet_addr(multicast_address),
                .sin_port        = htons(multicast_port),
            },
        .mreq.imr_multiaddr.s_addr = inet_addr(multicast_address),
        .mreq.imr_interface.s_addr = htonl(INADDR_ANY),
    };

    sys_thread_new("multicast_thread", udp_multicast_thread, config, 1024,
                   (osPriority_t)osPriorityNormal);
}
