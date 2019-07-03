#include "ai/world/game_state.h"

#include "ball.h"
#include "game_state.h"
#include "util/logger/init.h"


bool GameState::isHalted() const
{
    return state == HALT;
}

bool GameState::isStopped() const
{
    return state == STOP;
}

bool GameState::isPlaying() const
{
    return state == PLAYING;
}

bool GameState::isKickoff() const
{
    return restart_reason == KICKOFF;
}

bool GameState::isPenalty() const
{
    return restart_reason == PENALTY;
}

bool GameState::isBallPlacement() const
{
    return restart_reason == BALL_PLACEMENT;
}

bool GameState::isOurRestart() const
{
    // it has to be a restart for it to be our restart
    return our_restart && restart_reason != RestartReason::NONE;
}

bool GameState::isDirectFree() const
{
    return restart_reason == DIRECT;
}

bool GameState::isIndirectFree() const
{
    return restart_reason == INDIRECT;
}

bool GameState::isOurKickoff() const
{
    return isKickoff() && our_restart;
}

bool GameState::isOurPenalty() const
{
    return isPenalty() && our_restart;
}

bool GameState::isOurDirectFree() const
{
    return isDirectFree() && our_restart;
}

bool GameState::isOurIndirectFree() const
{
    return isIndirectFree() && our_restart;
}

bool GameState::isOurFreeKick() const
{
    return isOurDirectFree() || isOurIndirectFree();
}

bool GameState::isOurPlacement() const
{
    return isBallPlacement() && our_restart;
}

bool GameState::isTheirKickoff() const
{
    return isKickoff() && !our_restart;
}

bool GameState::isTheirPenalty() const
{
    return isPenalty() && !our_restart;
}

bool GameState::isTheirDirectFree() const
{
    return isDirectFree() && !our_restart;
}

bool GameState::isTheirIndirect() const
{
    return isIndirectFree() && !our_restart;
}

bool GameState::isTheirFreeKick() const
{
    return isTheirDirectFree() || isTheirIndirect();
}

bool GameState::isTheirBallPlacement() const
{
    return isBallPlacement() && !our_restart;
}

// Robots must be in position for a restart
bool GameState::isSetupRestart() const
{
    return state == SETUP || state == READY;
}

bool GameState::isSetupState() const
{
    return state == SETUP;
}

bool GameState::isReadyState() const
{
    return state == READY;
}

// One of our robots can kick the ball
bool GameState::canKick() const
{
    return state == PLAYING || (our_restart && state == READY);
}

bool GameState::stayAwayFromBall() const
{
    return state != PLAYING && !our_restart;
}

// Our robots must stay on our half of the field
bool GameState::stayOnSide() const
{
    return isSetupRestart() && restart_reason == KICKOFF && !our_restart;
}

// Our robots (except the penalty kicker) must stay a set distance behind the penalty line
bool GameState::stayBehindPenaltyLine() const
{
    return restart_reason == PENALTY;
}

void GameState::setBallPlacementPoint(Point placementPoint)
{
    ball_placement_point = placementPoint;
}

Point GameState::getBallPlacementPoint() const
{
    return ball_placement_point;
}

// apologies for this monster switch statement
void GameState::updateRefboxGameState(RefboxGameState gameState, const Ball &ball)
{
    if (gameState != game_state)
    {
        game_state = gameState;

        switch (gameState)
        {
            case RefboxGameState::HALT:
                state          = HALT;
                restart_reason = NONE;
                break;
            case RefboxGameState::STOP:
                state          = STOP;
                restart_reason = NONE;
                our_restart    = false;
                break;
            case RefboxGameState::NORMAL_START:
                state = READY;
                break;
            case RefboxGameState::FORCE_START:
                state          = PLAYING;
                restart_reason = NONE;
                break;
            case RefboxGameState::PREPARE_KICKOFF_US:
                state          = SETUP;
                restart_reason = KICKOFF;
                our_restart    = true;
                break;
            case RefboxGameState::PREPARE_KICKOFF_THEM:
                state          = SETUP;
                restart_reason = KICKOFF;
                our_restart    = false;
                break;
            case RefboxGameState::PREPARE_PENALTY_US:
                state          = SETUP;
                restart_reason = PENALTY;
                our_restart    = true;
                break;
            case RefboxGameState::PREPARE_PENALTY_THEM:
                state          = SETUP;
                restart_reason = PENALTY;
                our_restart    = false;
                break;
            case RefboxGameState::DIRECT_FREE_US:
                state          = PLAYING;
                restart_reason = DIRECT;
                our_restart    = true;
                break;
            case RefboxGameState::DIRECT_FREE_THEM:
                state          = READY;
                restart_reason = DIRECT;
                our_restart    = false;
                break;
            case RefboxGameState::INDIRECT_FREE_US:
                state          = PLAYING;
                restart_reason = INDIRECT;
                our_restart    = true;
                break;
            case RefboxGameState::INDIRECT_FREE_THEM:
                state          = READY;
                restart_reason = INDIRECT;
                our_restart    = false;
                break;
            case RefboxGameState::TIMEOUT_US:
                state          = HALT;
                restart_reason = NONE;
                break;
            case RefboxGameState::TIMEOUT_THEM:
                state          = HALT;
                restart_reason = NONE;
                break;
            case RefboxGameState::GOAL_US:
                state          = STOP;
                restart_reason = NONE;
                break;
            case RefboxGameState::GOAL_THEM:
                state          = STOP;
                restart_reason = NONE;
                break;
            case RefboxGameState::BALL_PLACEMENT_US:
                state          = SETUP;
                restart_reason = BALL_PLACEMENT;
                our_restart    = true;
                break;
            case RefboxGameState::BALL_PLACEMENT_THEM:
                state          = SETUP;
                restart_reason = BALL_PLACEMENT;
                our_restart    = false;
                break;
            default:
                LOG(WARNING) << "Unrecognized RefboxGameState" << std::endl;
                break;
        }
    }

    if (state == READY && restart_reason != PENALTY)
    {
        if (!ball_state)
        {
            // Save the ball state so we can tell once it moves
            ball_state = ball;
        }
        else if ((ball.position() - ball_state->position()).len() > 0.03)
        {
            // Once the ball has moved enough, the restart is finished
            setRestartCompleted();
            ball_state = std::nullopt;
        }
    }
}

RefboxGameState GameState::getRefboxGameState() const
{
    return game_state;
}

GameState::RestartReason GameState::getRestartReason() const
{
    return restart_reason;
}

void GameState::setRestartCompleted()
{
    state          = PLAYING;
//    restart_reason = NONE;
}

static std::unordered_map<GameState::State, std::string> state_name_map = {
        {GameState::State::HALT, "HALT"},
        {GameState::State::STOP, "STOP"},
        {GameState::State::SETUP, "SETUP"},
        {GameState::State::READY, "READY"},
        {GameState::State::PLAYING, "PLAYING"},
};

static std::unordered_map<GameState::RestartReason, std::string> restart_reason_name_map = {
        {GameState::RestartReason::NONE, "NONE"},
        {GameState::RestartReason::KICKOFF, "KICKOFF"},
        {GameState::RestartReason::DIRECT, "DIRECT"},
        {GameState::RestartReason::INDIRECT, "INDIRECT"},
        {GameState::RestartReason::PENALTY, "PENALTY"},
        {GameState::RestartReason::BALL_PLACEMENT, "BALL_PLACEMENT"},
};

std::ostream &operator<<(std::ostream &os, const GameState& gs) {
    os << "{ state: " << state_name_map[gs.state]
       << " restart_reason: " << restart_reason_name_map[gs.restart_reason]
       << " refbox_game_state: " << gs.game_state;
    if (gs.ball_state) {
        os << " ball_state: " << gs.ball_state->position();
    }
    os << " }";
    return os;
}
