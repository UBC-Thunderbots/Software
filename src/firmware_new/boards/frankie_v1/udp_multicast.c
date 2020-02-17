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


uint8_t buffer[5000];
int msg_count       = 0;
robot_ack ack       = robot_ack_init_zero;
control_msg control = control_msg_init_zero;

int udp_multicast_join_group(const char *multicast_ip, int multicast_port);
static void udp_multicast_thread(void *arg);

int udp_multicast_join_group(const char *multicast_ip, int multicast_port)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    struct sockaddr_in serv_addr;
    serv_addr.sin_family      = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(multicast_ip);
    serv_addr.sin_port        = htons(multicast_port);

    if (bind(sock, (struct sockaddr *)&serv_addr, (socklen_t)sizeof(serv_addr)) < 0)
        return -1;

    ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(multicast_ip);
    mreq.imr_interface.s_addr = INADDR_ANY;

    if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq)) < 0)
        return -1;
    return sock;
}

static void udp_multicast_thread(void *arg)
{
    int socket  = *(int *)arg;
    int counter = 0;

    while (1)
    {
        recv(socket, &buffer, 5000, 0);
        counter++;
    }
}

void udp_multicast_init(const char *multicast_address, int multicast_port)
{
    // create a connection to the multicast group
    int socket = udp_multicast_join_group(multicast_address, multicast_port);

    sys_thread_new("multicast_thread", udp_multicast_thread, &socket, 1024,
                   (osPriority_t)osPriorityNormal);
}
