#include "firmware/boards/robot_stm32h7/io/proto_multicast_communication_profile.h"

#include <stdlib.h>

#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

/**
 * ProtoMulticastCommunicationProfile_t contains the information required
 * to join a multicast group and then send or receive protobuf.
 */
typedef struct ProtoMulticastCommunicationProfile
{
    // name of the profile to pass down to other
    // elements that may require a name (mutex, etc..)
    const char* profile_name;

    // multicast group info: the address and the port to bind to and
    // join to send/recv proto
    ip_addr_t multicast_address;
    uint16_t port;

    // protobuf info: the pb_field_t array fully defines the protobuf_struct,
    // so we use a void pointer to be generic to any protobuf message.
    void* protobuf_struct;
    const pb_field_t* message_fields;
    uint16_t message_max_size;

    // communication_event: these events will be used to control when the networking
    // tasks run. The networking tasks will also signal certain events.
    osEventFlagsId_t communication_event;

    // mutex to protect the protobuf struct
    osMutexId_t profile_mutex;

} ProtoMulticastCommunicationProfile_t;

ProtoMulticastCommunicationProfile_t* io_proto_multicast_communication_profile_create(
    const char* profile_name, const char* multicast_address, uint16_t port,
    void* protobuf_struct, const pb_field_t* message_fields, uint16_t message_max_size)
{
    ProtoMulticastCommunicationProfile_t* profile =
        (ProtoMulticastCommunicationProfile_t*)malloc(
            sizeof(ProtoMulticastCommunicationProfile_t));

    const osMutexAttr_t mutex_attr = {profile_name, osMutexPrioInherit, NULL, 0U};

    profile->profile_name        = profile_name;
    profile->port                = port;
    profile->message_fields      = message_fields;
    profile->message_max_size    = message_max_size;
    profile->protobuf_struct     = protobuf_struct;
    profile->profile_mutex       = osMutexNew(&mutex_attr);
    profile->communication_event = osEventFlagsNew(NULL);
    ip6addr_aton(multicast_address, &profile->multicast_address);

    return profile;
}

void io_proto_multicast_communication_profile_destroy(
    ProtoMulticastCommunicationProfile_t* profile)
{
    osMutexDelete(profile->profile_mutex);
    free(profile);
}

const char* io_proto_multicast_communication_profile_getProfileName(
    ProtoMulticastCommunicationProfile_t* profile)
{
    return profile->profile_name;
}

const ip_addr_t* io_proto_multicast_communication_profile_getAddress(
    ProtoMulticastCommunicationProfile_t* profile)
{
    return &(profile->multicast_address);
}

uint16_t io_proto_multicast_communication_profile_getPort(
    ProtoMulticastCommunicationProfile_t* profile)
{
    return profile->port;
}

void* io_proto_multicast_communication_profile_getProtoStruct(
    ProtoMulticastCommunicationProfile_t* profile)
{
    return profile->protobuf_struct;
}

const pb_field_t* io_proto_multicast_communication_profile_getProtoFields(
    ProtoMulticastCommunicationProfile_t* profile)
{
    return profile->message_fields;
}

uint16_t io_proto_multicast_communication_profile_getMaxProtoSize(
    ProtoMulticastCommunicationProfile_t* profile)
{
    return profile->message_max_size;
}

void io_proto_multicast_communication_profile_acquireLock(
    ProtoMulticastCommunicationProfile_t* profile)
{
    osMutexAcquire(profile->profile_mutex, osWaitForever);
}

void io_proto_multicast_communication_profile_releaseLock(
    ProtoMulticastCommunicationProfile_t* profile)
{
    osMutexRelease(profile->profile_mutex);
}

void io_proto_multicast_communication_profile_notifyEvents(
    ProtoMulticastCommunicationProfile_t* profile, uint32_t events)
{
    osEventFlagsSet(profile->communication_event, events);
}

uint32_t io_proto_multicast_communication_profile_blockUntilEvents(
    ProtoMulticastCommunicationProfile_t* profile, uint32_t events)
{
    return osEventFlagsWait(profile->communication_event, events, osFlagsWaitAny,
                            osWaitForever);
}
