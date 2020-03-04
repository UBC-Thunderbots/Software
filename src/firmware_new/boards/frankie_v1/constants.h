#pragma once

// This file contains all constants that are shared between our software (AI)
// and firmware code. Since this needs to be compiled by both C and C++, everything
// should be defined in a way that's compatible with C.
//
// NOTE: This file is here until its moved to the shared folder after the new firmware
// is setup
//

// the IPv6 multicast address, only ff02 is important, the rest is random
// see https://en.wikipedia.org/wiki/Solicited-node_multicast_address for why ff02 matters
const char* AI_MULTICAST_ADDRESS = "ff02::c3d0:42d2:bb8";
// the port to send multicast packets for the AI (multicast)
const unsigned ROBOT_MULTICAST_LISTEN_PORT = 42000;
// the port that the AI is listening on for robot msgs (unicast)
const unsigned ROBOT_UNICAST_SEND_PORT = 42001;
// the port to listen to the multicast packets for the robot (multicast)
const unsigned AI_MULTICAST_SEND_PORT = ROBOT_MULTICAST_LISTEN_PORT;
// the port to send unicast packets to for robot msgs (unicast)
const unsigned AI_UNICAST_LISTEN_PORT = ROBOT_UNICAST_SEND_PORT;
