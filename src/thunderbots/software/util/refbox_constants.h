#pragma once

#include <ostream>

enum class RefboxGameState
{
    HALT = 0,
    STOP,
    NORMAL_START,
    FORCE_START,
    PREPARE_KICKOFF_US,
    PREPARE_KICKOFF_THEM,
    PREPARE_PENALTY_US,
    PREPARE_PENALTY_THEM,
    DIRECT_FREE_US,
    DIRECT_FREE_THEM,
    INDIRECT_FREE_US,
    INDIRECT_FREE_THEM,
    TIMEOUT_US,
    TIMEOUT_THEM,
    GOAL_US,
    GOAL_THEM,
    BALL_PLACEMENT_US,
    BALL_PLACEMENT_THEM,
    LAST_ENUM_ITEM_UNUSED
};

std::ostream& operator<<(std::ostream& os, const RefboxGameState& state);