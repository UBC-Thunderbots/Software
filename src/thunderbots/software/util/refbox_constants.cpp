#include "util/refbox_constants.h"

#include <map>

#include "refbox_constants.h"

static const std::map<RefboxGameState, std::string> refbox_game_state_names = {
    {RefboxGameState::HALT, "HALT"},
    {RefboxGameState::STOP, "STOP"},
    {RefboxGameState::NORMAL_START, "NORMAL_START"},
    {RefboxGameState::FORCE_START, "FORCE_START"},
    {RefboxGameState::PREPARE_KICKOFF_US, "PREPARE_KICKOFF_US"},
    {RefboxGameState::PREPARE_KICKOFF_THEM, "PREPARE_KICKOFF_THEM"},
    {RefboxGameState::PREPARE_PENALTY_US, "PREPARE_PENALTY_US"},
    {RefboxGameState::PREPARE_PENALTY_THEM, "PREPARE_PENALTY_THEM"},
    {RefboxGameState::DIRECT_FREE_US, "DIRECT_FREE_US"},
    {RefboxGameState::DIRECT_FREE_THEM, "DIRECT_FREE_THEM"},
    {RefboxGameState::INDIRECT_FREE_US, "INDIRECT_FREE_US"},
    {RefboxGameState::INDIRECT_FREE_THEM, "INDIRECT_FREE_THEM"},
    {RefboxGameState::TIMEOUT_US, "TIMEOUT_US"},
    {RefboxGameState::TIMEOUT_THEM, "TIMEOUT_THEM"},
    {RefboxGameState::GOAL_US, "GOAL_US"},
    {RefboxGameState::GOAL_THEM, "GOAL_THEM"},
    {RefboxGameState::BALL_PLACEMENT_US, "BALL_PLACEMENT_US"},
    {RefboxGameState::BALL_PLACEMENT_THEM, "BALL_PLACEMENT_THEM"},
    {RefboxGameState::REFBOX_GAME_STATE_COUNT, "REFBOX_GAME_STATE_COUNT"}};

std::ostream& operator<<(std::ostream& os, const RefboxGameState& state)
{
    if (refbox_game_state_names.find(state) != refbox_game_state_names.end())
    {
        os << refbox_game_state_names.find(state)->second;
    }
    else
    {
        os << "INVALID RefboxGameState: " << static_cast<int>(state);
    }
    return os;
}
