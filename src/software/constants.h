#pragma once

#include <string>

namespace Util
{
    namespace Constants
    {
        // Networking and vision
        static const std::string SSL_VISION_DEFAULT_MULTICAST_ADDRESS = "224.5.23.2";
        static const unsigned short SSL_VISION_MULTICAST_PORT         = 10020;

        // GrSim networking and communication
        // TODO: BETTER NAMES
        static const std::string GRSIM_COMMAND_NETWORK_ADDRESS = "127.0.0.1";
        static const short GRSIM_COMMAND_NETWORK_PORT          = 20011;

        // Refbox address
        static const std::string SSL_GAMECONTROLLER_MULTICAST_ADDRESS = "224.5.23.1";
        static const unsigned short SSL_GAMECONTROLLER_MULTICAST_PORT = 10003;

        // There are 4 cameras for SSL Division B
        static const unsigned int NUMBER_OF_SSL_VISION_CAMERAS = 4;

        // How many milliseconds a robot must not be seen in vision before it is
        // considered as "gone" and no longer reported.
        static const unsigned int ROBOT_DEBOUNCE_DURATION_MILLISECONDS = 200;
    }  // namespace Constants
}  // namespace Util
