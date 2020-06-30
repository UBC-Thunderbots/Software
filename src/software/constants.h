#pragma once

#include <string>

// GrSim networking and communication
// TODO: BETTER NAMES
static const std::string GRSIM_COMMAND_NETWORK_ADDRESS = "127.0.0.1";
static constexpr short GRSIM_COMMAND_NETWORK_PORT      = 20011;

// There are 4 cameras for SSL Division B
static constexpr unsigned int NUMBER_OF_SSL_VISION_CAMERAS = 4;

// How many milliseconds a robot must not be seen in vision before it is
// considered as "gone" and no longer reported.
static constexpr unsigned int ROBOT_DEBOUNCE_DURATION_MILLISECONDS = 200;
