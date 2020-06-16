#pragma once

#include "pb.h"

typedef struct ProtoMulticastCommunicationProfile ProtoMulticastCommunicationProfile_t;

/**
 * EVENTS
 */
typedef enum 
{
    LWIP_NETIF_UP   = 1 << 0,  // network interface link up
    UPDATED_PROTO   = 1 << 1,  // protobuf struct has been updated
    RECEIVED_PROTO  = 1 << 2,  // new protobuf has been received
    RECEIVE_TIMEOUT = 1 << 3,  // waiting takes longer than NETWORK_TIMEOUT_MILLISECONDS
} ProtofMulticastCommunicationEvent_t ;

/***
 * TASKS:
 *
 *                    Sender Task                             Listener Task
 *         +-------------------------------+         +------------------------------+
 *         |                               |         |                              |
 *         |  wait for LWIP_NETIF_UP       |         |  wait for LWIP_NETIF_UP      |
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
 *     +----+ send buffer to group         |     +----+ notify PROTO_RECEIVED       |
 *         |                               |         |                              |
 *         +-------------------------------+         +------------------------------+
 *
 */
void io_proto_multicast_sender_Task(void* arg);
void io_proto_multicast_listener_Task(void* arg);

/**
 * Create an ProtoMulticastCommunicationProfile
 *
 * Contains all the networking and protobuf information required to join a multicast
 * group and communicate protobuf
 *
 * @param multicast_address [in] The multicast channel the robot should join
 * @param port [in] The port to bind to
 * @param protobuf_struct [in/out] The serialized proto in the incoming network packet,
 *        will get deserialized into this struct in the listener task. The sender task
 *        will serialize the protobuf_struct and send it over the network.
 * @param message_fields [in] The nanopb message fields used to seriallize/deserialize
 * @param message_max_size [in] The maximum known size of the protobuf.
 *
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
 * @param ProtoMulticastSender/ListenerProfile_t the profile to lock
 */
void io_proto_multicast_communication_profile_acquireLock(
    ProtoMulticastCommunicationProfile_t* communication_profile);

/**
 * Release the profile lock.
 *
 * @pre lock must be acquired through the communication_profile_acquireLock call
 * @param profile The profile to release the mutex for
 */
void io_proto_multicast_communication_profile_releaseLock(
    ProtoMulticastCommunicationProfile_t* communication_profile);

/**
 * Signal the proto_multicast_sender_Task that the protobuf is ready
 * to be serialized and sent over the network.
 *
 * @pre must have called sender_profile_releaseLock
 * @param sender_profile The profile used to notify the corresponding sending task
 * @param event The event to send to all tasks using the communication profile
 */
void io_proto_multicast_communication_profile_notifyEvent(
    ProtoMulticastCommunicationProfile_t* communication_profile,
    ProtofMulticastCommunicationEvent_t event);

/**
 * Block the calling function/task until a new protobuf value has been
 * received over the network.
 *
 * @pre must have called listener_profile_releaseLock
 * @param communication_profile The profile that the notification will occur on
 * @param event The event to wait block on
 */
void io_proto_multicast_communication_profile_blockUntilEvent(
    ProtoMulticastCommunicationProfile_t* communication_profile,
    ProtofMulticastCommunicationEvent_t event);

/**
 * Destroy the given ProtoMulticastCommunicationProfile_t
 *
 * @param communication_profile The profile to delete
 */
void io_proto_multicast_communication_profile_destroy(
    ProtoMulticastCommunicationProfile_t* communication_profile);
