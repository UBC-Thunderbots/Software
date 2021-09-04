#pragma once

#include <string>

// How many milliseconds a robot must not be seen in vision before it is
// considered as "gone" and no longer reported.
static constexpr unsigned int ROBOT_DEBOUNCE_DURATION_MILLISECONDS = 200;


static constexpr unsigned int MAX_SIMULATOR_MULTICAST_CHANNELS = 16;

// Networking
// the IPv6 multicast address, only ff02 is important, the rest is random
// see https://en.wikipedia.org/wiki/Solicited-node_multicast_address for why ff02 matters
static const std::string SIMULATOR_MULTICAST_CHANNELS[MAX_SIMULATOR_MULTICAST_CHANNELS] =
    {
        "ff02::c3d0:42d2:cc01", "ff02::c3d0:42d2:cc02", "ff02::c3d0:42d2:cc03",
        "ff02::c3d0:42d2:cc04", "ff02::c3d0:42d2:cc05", "ff02::c3d0:42d2:cc06",
        "ff02::c3d0:42d2:cc07", "ff02::c3d0:42d2:cc08", "ff02::c3d0:42d2:cc09",
        "ff02::c3d0:42d2:cc10", "ff02::c3d0:42d2:cc11", "ff02::c3d0:42d2:cc12",
        "ff02::c3d0:42d2:cc13", "ff02::c3d0:42d2:cc14", "ff02::c3d0:42d2:cc15",
        "ff02::c3d0:42d2:cc16",
};

static const std::string NETWORK_LOGGING_MULTICAST_CHANNELS[MAX_SIMULATOR_MULTICAST_CHANNELS] =
    {
        "ff02::c3d0:42d3:cc01", "ff02::c3d0:42d3:cc02", "ff02::c3d0:42d3:cc03",
        "ff02::c3d0:42d3:cc04", "ff02::c3d0:42d3:cc05", "ff02::c3d0:42d3:cc06",
        "ff02::c3d0:42d3:cc07", "ff02::c3d0:42d3:cc08", "ff02::c3d0:42d3:cc09",
        "ff02::c3d0:42d3:cc10", "ff02::c3d0:42d3:cc11", "ff02::c3d0:42d3:cc12",
        "ff02::c3d0:42d3:cc13", "ff02::c3d0:42d3:cc14", "ff02::c3d0:42d3:cc15",
        "ff02::c3d0:42d3:cc16",
    };

const short unsigned int NETWORK_LOGS_PORT   = 42079;
