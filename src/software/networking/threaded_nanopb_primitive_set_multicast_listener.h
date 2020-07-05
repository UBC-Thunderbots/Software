#pragma once

#include "software/networking/nanopb_primitive_set_multicast_listener.h"
#include "software/networking/threaded_network_listener.h"

/**
 * A threaded listener that receives serialized PrimitiveSetMsg's over the network
 */
using ThreadedNanoPbPrimitiveSetMulticastListener =
    ThreadedNetworkListener<NanoPbPrimitiveSetMulticastListener, PrimitiveSetMsg>;
