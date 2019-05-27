#pragma once

#include <string>

#include "../ai/world/team.h"

namespace Util
{
    namespace Constants
    {
        // Constants for ROS nodes, message, and topics
        static const std::string NETWORK_INPUT_BALL_TOPIC  = "backend/ball";
        static const std::string NETWORK_INPUT_FIELD_TOPIC = "backend/field";
        static const std::string NETWORK_INPUT_FRIENDLY_TEAM_TOPIC =
            "backend/friendly_team";
        static const std::string NETWORK_INPUT_ENEMY_TEAM_TOPIC = "backend/enemy_team";
        static const std::string NETWORK_INPUT_GAMECONTROLLER_TOPIC =
            "backend/gamecontroller";
        static const std::string NETWORK_INPUT_WORLD_TOPIC   = "backend/world";
        static const std::string AI_PRIMITIVES_TOPIC         = "backend/primitives";
        static const std::string ROBOT_MESSAGE_TOPIC         = "log/robot_message";
        static const std::string ROBOT_STATUS_TOPIC          = "backend/robot_status";
        static const std::string VISUALIZER_DRAW_LAYER_TOPIC = "visualizer/layers";
        // The topic published by the joy_node that contains information about any plugged
        // in joysticks / controllers
        static const std::string JOY_NODE_TOPIC = "joy";

        // TODO: Make this a tuneable parameter
        static const TeamColour FRIENDLY_TEAM_COLOUR = YELLOW;

        // Networking and vision
        static const std::string SSL_VISION_MULTICAST_ADDRESS = "224.5.23.2";
        static const unsigned short SSL_VISION_MULTICAST_PORT = 10020;

        // GrSim networking and communication
        // TODO: BETTER NAMES
        static const std::string GRSIM_COMMAND_NETWORK_ADDRESS = "127.0.0.1";
        static const short GRSIM_COMMAND_NETWORK_PORT          = 20011;

        // Refbox address
        static const std::string SSL_GAMECONTROLLER_MULTICAST_ADDRESS = "224.5.23.1";
        static const unsigned short SSL_GAMECONTROLLER_MULTICAST_PORT = 10003;

        // There are 4 cameras for SSL Division B
        static const unsigned int NUMBER_OF_SSL_VISION_CAMERAS = 4;

        // Canvas messenger message maximum publishing frequency
        static const unsigned int DESIRED_CANVAS_MESSAGE_FREQ = 60;

        // How many milliseconds a robot must not be seen in vision before it is
        // considered as "gone" and no longer reported.
        static const unsigned int ROBOT_DEBOUNCE_DURATION_MILLISECONDS = 200;
    }  // namespace Constants
}  // namespace Util
