#pragma once

#include <string>

// How many milliseconds a robot must not be seen in vision before it is
// considered as "gone" and no longer reported.
static constexpr unsigned int ROBOT_DEBOUNCE_DURATION_MILLISECONDS = 200;


static constexpr unsigned int MAX_SIMULATOR_MULTICAST_CHANNELS = 2;

// Networking
// the IPv6 multicast address, only ff02 is important, the rest is random
// see https://en.wikipedia.org/wiki/Solicited-node_multicast_address for why ff02 matters
static const std::string SIMULATOR_MULTICAST_CHANNELS[MAX_SIMULATOR_MULTICAST_CHANNELS] =
    {"224.6.6.6", "224.6.6.7"};
