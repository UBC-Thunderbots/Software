
/**
 ******************************************************************************
 * @file    LwIP/LwIP_UDP_Echo_Server/Src/udp_echoserver.c
 * @author  MCD Application Team
 * @brief   UDP echo server
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "udp_echoserver.h"

#include <stdio.h>
#include <string.h>

#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "main.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "shared/proto/control_fw.pb.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// global proto msg that will be updated to the most recent msg sent
// TODO remove this file as part of RTOS upgrade + add API
control_msg control = control_msg_init_zero;
robot_ack ack       = robot_ack_init_zero;
uint8_t buffer[robot_ack_size];
uint32_t msg_count = 0;

/* Private function prototypes -----------------------------------------------*/
void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                                     const ip_addr_t *addr, u16_t port);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initialize the server application.
 * @param  None
 * @retval None
 */
void udp_echoserver_init(void)
{
    struct udp_pcb *upcb;
    err_t err;

    /* Create a new UDP control block  */
    upcb = udp_new();

    if (upcb)
    {
        /* Bind the upcb to the UDP_PORT port */
        /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
        err = udp_bind(upcb, IP_ADDR_ANY, 42069);

        if (err == ERR_OK)
        {
            /* Set a receive callback for the upcb */
            udp_recv(upcb, udp_echoserver_receive_callback, NULL);
        }
        else
        {
            udp_remove(upcb);
        }
    }
}

/**
 * @brief This function is called when an UDP datagrm has been received on the port
 * UDP_PORT.
 * @param arg user supplied argument (udp_pcb.recv_arg)
 * @param pcb the udp_pcb which received data
 * @param p the packet buffer that was received
 * @param addr the remote IP address from which the packet was received
 * @param port the remote port from which the packet was received
 * @retval None
 */
void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                                     const ip_addr_t *addr, u16_t port)
{
    struct pbuf *p_tx;

    /* allocate pbuf from RAM*/
    p_tx = pbuf_alloc(PBUF_TRANSPORT, p->len, PBUF_RAM);

    if (p_tx != NULL)
    {
        pbuf_take(p_tx, (char *)p->payload, p->len);

        // Create a stream that reads from the buffer
        pb_istream_t in_stream = pb_istream_from_buffer((uint8_t *)p->payload, p->len);

        if (pb_decode(&in_stream, control_msg_fields, &control))
        {
            pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

            ack.msg_count = msg_count++;

            pb_encode(&stream, robot_ack_fields, &ack);

            p_tx->payload = buffer;

            /* Connect to the remote client */
            udp_connect(upcb, IP_ADDR_BROADCAST, 42069);

            /* Tell the client that we have accepted it */
            udp_send(upcb, p_tx);

            /* free the UDP connection, so we can accept new clients */
            udp_disconnect(upcb);
        }

        /* Free the p_tx buffer */
        pbuf_free(p_tx);

        /* Free the p buffer */
        pbuf_free(p);
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
