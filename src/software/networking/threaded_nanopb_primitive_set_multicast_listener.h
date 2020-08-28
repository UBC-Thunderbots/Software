#pragma once

#include "software/networking/nanopb_primitive_set_multicast_listener.h"
#include "software/networking/threaded_network_listener.h"

/**
 * A threaded listener that receives serialized `PrimitiveSet`s over the network
 */
using ThreadedNanoPbPrimitiveSetMulticastListener =
    ThreadedMulticastListener<NanoPbPrimitiveSetMulticastListener,
                              TbotsProto_PrimitiveSet>;
