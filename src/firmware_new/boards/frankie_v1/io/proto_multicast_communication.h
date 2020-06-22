#pragma once

#include <pb.h>

typedef struct ProtoMulticastCommunicationProfile ProtoMulticastCommunicationProfile_t;

/**
 * Event Flags: masks used to signal tasks to unblock and run specific actions
 */
typedef enum
{
    PROTO_UPDATED   = 1 << 0,  // protobuf struct has been updated
    RECEIVED_PROTO  = 1 << 1,  // new protobuf has been received
    RECEIVE_TIMEOUT = 1 << 2,  // blocking takes longer than NETWORK_TIMEOUT_MS
} ProtoMulticastCommunicationEventFlags_t;

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
 * @param arg Unused
 */
void io_proto_multicast_startNetworkingTask(void* arg);

/**
 * Initializes the proto_multicast_communication library
 */
void io_proto_multicast_communication_init(void);

/**
 * Create an ProtoMulticastCommunicationProfile
 *
 * Contains all the networking and protobuf information required to join a multicast
 * group and communicate protobuf
 *
 * @param multicast_address [in] The multicast channel the robot should join
 * @param port [in] The port to bind to
 * @param message_fields [in] The nanopb message fields used to seriallize/deserialize
 * @param message_max_size [in] The maximum known size of the protobuf.
 * @param protobuf_struct [in/out] The serialized proto in the incoming network packet,
 *        will get deserialized into this struct in the listener task. The sender task
 *        will serialize the protobuf_struct and send it over the network.
 * @return ProtoMulticastCommunicationProfile_t the profile to provide to the task
 *
 */
ProtoMulticastCommunicationProfile_t* io_proto_multicast_communication_profile_create(
    const char* profile_name, const char* multicast_address, uint16_t port,
    void* protobuf_struct, const pb_field_t* message_fields, uint16_t message_max_size);

/**
 * Locks the profile to implicitly lock the internally tracked pointer
 * to the protobuf_struct. The respective profile lock must be acquired before
 * reading/writing the values inside the proto struct.
 *
 * NOTE: call blocks until the lock is acquired. If there is a higher
 * priority task waiting for the mutex, the task holding this lock will
 * inherit the waiting tasks priority. (osMutexPrioInherit)
 *
 * @param communication_profile The profile to lock
 */
void io_proto_multicast_communication_profile_acquireLock(
    ProtoMulticastCommunicationProfile_t* communication_profile);

/**
 * Release the profile lock.
 *
 * @pre lock must be acquired through the call to "communication_profile_acquireLock"
 *
 * @param communication_profile The profile to release the mutex for
 */
void io_proto_multicast_communication_profile_releaseLock(
    ProtoMulticastCommunicationProfile_t* communication_profile);

/**
 * Signal the tasks waiting on the communication_profile with the provided events.
 *
 * @pre do NOT hold the profile lock, this will cause everything to grind to a halt
 *
 * @param communication_profile The profile that the notification will be "sent on". Any
 * task blocking with io_proto_multicast_communication_profile_blockUntilEvents will be
 * notified.
 * @param events The events to notify, bitwise or of
 * ProtoMulticastCommunicationEventFlags_t
 */
void io_proto_multicast_communication_profile_notifyEvents(
    ProtoMulticastCommunicationProfile_t* communication_profile, uint32_t events);

/**
 * Block the calling function until a new protobuf value has been received over the
 * network.
 *
 * @pre do NOT hold the profile lock, this will cause everything to grind to a halt
 *
 * @param communication_profile The profile that the notification will occur on
 * @param events The events to block on, bitwise or of
 * ProtoMulticastCommunicationEventFlags_t
 */
uint32_t io_proto_multicast_communication_profile_blockUntilEvents(
    ProtoMulticastCommunicationProfile_t* communication_profile, uint32_t events);

/**
 * Destroy the given ProtoMulticastCommunicationProfile_t
 *
 * @param communication_profile The profile to delete
 */
void io_proto_multicast_communication_profile_destroy(
    ProtoMulticastCommunicationProfile_t* communication_profile);
