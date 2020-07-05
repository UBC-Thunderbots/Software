#pragma once

#include "software/networking/proto_multicast_listener.h"
#include "software/networking/threaded_network_listener.h"

/**
 * A threaded listener that receives serialized ReceiveProtoT Proto's over the network
 */
template <class ReceiveProtoT>
using ThreadedProtoMulticastListener =
    ThreadedNetworkListener<ProtoMulticastListener<ReceiveProtoT>, ReceiveProtoT>;
