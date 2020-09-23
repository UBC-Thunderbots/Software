#pragma once

#include <pb.h>

/***
 * TASKS:
 *
 *                    Sender Task                             Listener Task
 *         +-------------------------------+         +------------------------------+
 *         |                               |         |                              |
 *         |  wait for NETIF_CONFIGURED    |         |  wait for NETIF_CONFIGURED   |
 *         |                               |         |                              |
 *         |  join multicast group         |         |  join multicast group        |
 *         |                               |         |                              |
 *     +----> wait for PROTO_UPDATED event |     +----> wait for new proto on netif |
 *     |   |                               |     |   |                              |
 *     |   |  acquire lock                 |     |   |  acquire lock                |
 *     |   |                               |     |   |                              |
 *     |   |  serialize proto to buffer    |     |   |  deserialize buffer to proto |
 *     |   |                               |     |   |                              |
 *     |   |  release lock                 |     |   |  release lock                |
 *     |   |                               |     |   |                              |
 *     +----+ send buffer to group         |     +----+ notify RECEIVED_PROTO event |
 *         |                               |         |                              |
 *         +-------------------------------+         +------------------------------+
 *
 * @param communication_profile The pointer to the allocated communication profile
 * to use for the Task
 *
 * NOTE: These functions must run as a new Task
 */
void io_proto_multicast_sender_task(void* communication_profile);
void io_proto_multicast_listener_task(void* communication_profile);

/**
 * LWIP automatically places MX_LWIP_Init() in a default task. That function
 * call internally creates another TCP/IP task, and the default task just loops
 * forever.
 *
 * Cube forces us to generate this function, in main.c where we mark it as __weak and
 * define the implementation in this library (proto_multicast_communication). When the
 * proto communication library is linked w/ the main frankie_v1 binary, the __weak
 * reference is resolved to the "stronger" reference to the function defined below.
 *
 * We take over control so we can signal other networking tasks when MX_LWIP_Init()
 * is finished running and the network link is configured and up.
 *
 * @param unused
 */
void io_proto_multicast_startNetworkingTask(void* unused);

/**
 * Initializes the proto_multicast_communication library
 *
 * @param net_timeout_msg How long the receivers should wait before signaling
 *        a timeout event in ms
 */
void io_proto_multicast_communication_init(int net_timeout_ms);
