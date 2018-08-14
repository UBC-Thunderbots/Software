#pragma once

#include <string>
#include "../ai/world/team.h"

namespace UTIL
{
namespace CONSTANTS
{
// Constants for ROS nodes, message, and topics
static const std::string BACKEND_INPUT_BALL_TOPIC          = "backend/ball";
static const std::string BACKEND_INPUT_FIELD_TOPIC         = "backend/field";
static const std::string BACKEND_INPUT_FRIENDLY_TEAM_TOPIC = "backend/friendly_team";
static const std::string BACKEND_INPUT_ENEMY_TEAM_TOPIC    = "backend/enemy_team";
static const std::string AI_PRIMITIVES_TOPIC               = "backend/primitives";

// TODO: Make this a tuneable parameter
const TeamColour FRIENDLY_TEAM_COLOUR = YELLOW;
}
}
