#include "udp_multicast.h"

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
    /*// this buffer is used to hold serialized proto*/
    /*uint8_t buffer[RobotAck_size];*/

    /*while (1)*/
    /*{*/
    /*err = netconn_recv(recvconn, &rx_buf);*/

    /*if (err == ERR_OK)*/
    /*{*/
    /*tx_buf = netbuf_new();*/
    /*netbuf_alloc(tx_buf, RobotAck_size);*/

    /*// Create a stream that reads from the buffer*/
    /*pb_istream_t in_stream =*/
    /*pb_istream_from_buffer((uint8_t *)rx_buf->p->payload, rx_buf->p->tot_len);*/

    /*if (pb_decode(&in_stream, ControlMsg_fields, &control))*/
    /*{*/
    /*// update proto*/
    /*ack.msg_count = msg_count++;*/

    /*// serialize proto*/
    /*pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));*/
    /*pb_encode(&stream, RobotAck_fields, &ack);*/

    /*// package payload and send over udp*/
    /*tx_buf->p->payload = buffer;*/
    /*netconn_sendto(sendconn, tx_buf, (const ip_addr_t *)&(rx_buf->addr),*/
    /*config->send_port);*/
    /*}*/
    /*netbuf_delete(tx_buf);*/
    /*}*/
    /*netbuf_delete(rx_buf);*/
    /*}*/
}

void udp_multicast_init(const char *multicast_address, unsigned multicast_port,
                        unsigned send_port)
{
    /*multicast_config_t *config = malloc(sizeof(multicast_config_t));*/

    /**config = (multicast_config_t){*/
    /*.multicast_port = multicast_port,*/
    /*.send_port      = send_port,*/
    /*};*/

    /*ip6addr_aton(multicast_address, &config->multicast_address);*/

    /*sys_thread_new("multicast_thread", blocking_udp_multicast_loop, config, 1024,*/
    /*(osPriority_t)osPriorityNormal);*/
}
