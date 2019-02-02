#include "ai/world/game_state.h"

#include "game_state.h"


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

bool GameState::isOurDirect() const
{
    return isDirectFree() && our_restart;
}

bool GameState::isOurIndirect() const
{
    return isIndirectFree() && our_restart;
}

bool GameState::isOurFreeKick() const
{
    return isOurDirect() || isOurIndirect();
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

bool GameState::isTheirDirect() const
{
    return isDirectFree() && !our_restart;
}

bool GameState::isTheirIndirect() const
{
    return isIndirectFree() && !our_restart;
}

bool GameState::isTheirFreeKick() const
{
    return isTheirDirect() || isTheirIndirect();
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
void GameState::updateRefboxGameState(RefboxGameState gameState)
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
            state          = SETUP;
            restart_reason = DIRECT;
            our_restart    = true;
            break;
        case RefboxGameState::DIRECT_FREE_THEM:
            state          = SETUP;
            restart_reason = DIRECT;
            our_restart    = false;
            break;
        case RefboxGameState::INDIRECT_FREE_US:
            state          = SETUP;
            restart_reason = INDIRECT;
            our_restart    = true;
            break;
        case RefboxGameState::INDIRECT_FREE_THEM:
            state          = SETUP;
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
    restart_reason = NONE;
}
