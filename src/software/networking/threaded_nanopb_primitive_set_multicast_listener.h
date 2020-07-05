#pragma once

#include "software/networking/nanopb_primitive_set_multicast_listener.h"
#include "software/networking/threaded_network_listener.h"

// TODO: does it _really_ make sense to have these headers with just a typedef?

using ThreadedNanoPbPrimitiveSetMulticastListener =
    ThreadedNetworkListener<NanoPbPrimitiveSetMulticastListener, std::map<RobotId, PrimitiveMsg>>;

