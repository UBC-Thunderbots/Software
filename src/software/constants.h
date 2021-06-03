#pragma once

#include <string>

// How many milliseconds a robot must not be seen in vision before it is
// considered as "gone" and no longer reported.
static constexpr unsigned int ROBOT_DEBOUNCE_DURATION_MILLISECONDS = 200;

// Networking
// the IPv6 multicast address, only ff02 is important, the rest is random
// see https://en.wikipedia.org/wiki/Solicited-node_multicast_address for why ff02 matters
const char SIMULATOR_MULTICAST_CHANNELS[MAX_MULTICAST_CHANNELS][MULTICAST_CHANNEL_LENGTH] = {
        "ff02::c3d0:42d2:cc01", "ff02::c3d0:42d2:cc02", "ff02::c3d0:42d2:cc03",
        "ff02::c3d0:42d2:cc04", "ff02::c3d0:42d2:cc05", "ff02::c3d0:42d2:cc06",
        "ff02::c3d0:42d2:cc07", "ff02::c3d0:42d2:cc08", "ff02::c3d0:42d2:cc09",
        "ff02::c3d0:42d2:cc10", "ff02::c3d0:42d2:cc11", "ff02::c3d0:42d2:cc12",
        "ff02::c3d0:42d2:cc13", "ff02::c3d0:42d2:cc14", "ff02::c3d0:42d2:cc15",
        "ff02::c3d0:42d2:cc16",
};