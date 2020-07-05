#pragma once

#include "software/networking/proto_multicast_listener.h"
#include "software/networking/threaded_network_listener.h"

template <class ReceiveProto>
using ThreadedProtoMulticastListener =
    ThreadedNetworkListener<ProtoMulticastListener<ReceiveProto>, ReceiveProto>;

