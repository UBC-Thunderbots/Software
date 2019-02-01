#pragma once

#include <ostream>

enum class RefboxGameState
{
    // these enum items map to the constants in RefboxCommand.msg
    HALT                 = 0,
    STOP                 = 1,
    NORMAL_START         = 2,
    FORCE_START          = 3,
    PREPARE_KICKOFF_US   = 4,
    PREPARE_KICKOFF_THEM = 5,
    PREPARE_PENALTY_US   = 6,
    PREPARE_PENALTY_THEM = 7,
    DIRECT_FREE_US       = 8,
    DIRECT_FREE_THEM     = 9,
    INDIRECT_FREE_US     = 10,
    INDIRECT_FREE_THEM   = 11,
    TIMEOUT_US           = 12,
    TIMEOUT_THEM         = 13,
    GOAL_US              = 14,
    GOAL_THEM            = 15,
    BALL_PLACEMENT_US    = 16,
    BALL_PLACEMENT_THEM  = 17,
    REFBOX_GAME_STATE_COUNT
};

std::ostream& operator<<(std::ostream& os, const RefboxGameState& state);