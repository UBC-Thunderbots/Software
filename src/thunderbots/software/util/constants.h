#pragma once

#include <string>
#include "ai/world/field.h"
#include "ai/world/team.h"

namespace Util
{
    namespace Constants
    {
        // Constants for ROS nodes, message, and topics
        static const std::string BACKEND_INPUT_BALL_TOPIC  = "backend/ball";
        static const std::string BACKEND_INPUT_FIELD_TOPIC = "backend/field";
        static const std::string BACKEND_INPUT_FRIENDLY_TEAM_TOPIC =
            "backend/friendly_team";
        static const std::string BACKEND_INPUT_ENEMY_TEAM_TOPIC = "backend/enemy_team";
        static const std::string AI_PRIMITIVES_TOPIC            = "backend/primitives";

        // TODO: Make this a tuneable parameter
        static const TeamColour FRIENDLY_TEAM_COLOUR = TeamColour::YELLOW;

        // TODO: Make this a tunable parameter
        static const FieldSide FRIENDLY_FIELD_SIDE = TeamSide::WEST;

        // Networking and vision
        static const std::string SSL_VISION_MULTICAST_ADDRESS = "224.5.23.2";
        static const unsigned short SSL_VISION_MULTICAST_PORT = 10020;

        // Refbox address
        static const std::string SSL_GAMECONTROLLER_MULTICAST_ADDRESS = "224.5.23.1";
        static const unsigned short SSL_GAMECONTROLLER_MULTICAST_PORT = 10003;
    }  // namespace Constants
}  // namespace Util
