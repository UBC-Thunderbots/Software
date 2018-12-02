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
    return restart == KICKOFF;
}

bool GameState::isPenalty() const
{
    return restart == PENALTY;
}

bool GameState::isBallPlacement() const
{
    return restart == BALL_PLACEMENT;
}

bool GameState::isOurRestart() const
{
    return our_restart;
}

bool GameState::isDirectFree() const
{
    return restart == DIRECT;
}

bool GameState::isIndirectFree() const
{
    return restart == INDIRECT;
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

bool GameState::isTheirDirectFree() const
{
    return isDirectFree() && !our_restart;
}

bool GameState::isTheirIndirectFree() const
{
    return isIndirectFree() && !our_restart;
}

bool GameState::isTheirFreeKick() const
{
    return isTheirDirectFree() || isTheirIndirectFree();
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
    return state == PLAYING || (our_restart && state == PLAYING);
}

bool GameState::stayAwayFromBall() const
{
    return state != PLAYING && !our_restart;
}

// Our robots must stay on our half of the field
bool GameState::stayOnSide() const
{
    return isSetupRestart() && restart == KICKOFF;
}

// Our robots (except the penalty kicker) must stay 400mm behind the penalty
// line
bool GameState::stayBehindPenaltyLine() const
{
    return restart == PENALTY;
}

void GameState::setBallPlacementPoint(Point placementPoint)
{
    ballPlacementPoint = placementPoint;
}

Point GameState::getBallPlacementPoint() const
{
    return ballPlacementPoint;
}

// apologies for this monster switch statement
void GameState::updateRefboxGameState(RefboxGameState gameState)
{
    switch (gameState)
    {
        case RefboxGameState::HALT:
            state   = HALT;
            restart = NONE;
            break;
        case RefboxGameState::STOP:
            state       = STOP;
            restart     = NONE;
            our_restart = false;
            break;
        case RefboxGameState::NORMAL_START:
            state = PLAYING;
            break;
        case RefboxGameState::FORCE_START:
            state   = PLAYING;
            restart = NONE;
            break;
        case RefboxGameState::PREPARE_KICKOFF_US:
            state       = SETUP;
            restart     = KICKOFF;
            our_restart = true;
            break;
        case RefboxGameState::PREPARE_KICKOFF_THEM:
            state       = READY;
            restart     = KICKOFF;
            our_restart = false;
            break;
        case RefboxGameState::PREPARE_PENALTY_US:
            state       = SETUP;
            restart     = PENALTY;
            our_restart = true;
            break;
        case RefboxGameState::PREPARE_PENALTY_THEM:
            state       = READY;
            restart     = PENALTY;
            our_restart = false;
            break;
        case RefboxGameState::DIRECT_FREE_US:
            state       = SETUP;
            restart     = DIRECT;
            our_restart = true;
            break;
        case RefboxGameState::DIRECT_FREE_THEM:
            state       = READY;
            restart     = DIRECT;
            our_restart = false;
            break;
        case RefboxGameState::INDIRECT_FREE_US:
            state       = SETUP;
            restart     = INDIRECT;
            our_restart = true;
            break;
        case RefboxGameState::INDIRECT_FREE_THEM:
            state       = READY;
            restart     = INDIRECT;
            our_restart = false;
            break;
        case RefboxGameState::TIMEOUT_US:
            state   = HALT;
            restart = NONE;
            break;
        case RefboxGameState::TIMEOUT_THEM:
            state   = HALT;
            restart = NONE;
            break;
        case RefboxGameState::GOAL_US:
            state   = STOP;
            restart = NONE;
            break;
        case RefboxGameState::GOAL_THEM:
            state   = STOP;
            restart = NONE;
            break;
        case RefboxGameState::BALL_PLACEMENT_US:
            state       = READY;
            restart     = BALL_PLACEMENT;
            our_restart = true;
            break;
        case RefboxGameState::BALL_PLACEMENT_THEM:
            state       = SETUP;
            restart     = BALL_PLACEMENT;
            our_restart = false;
            break;
    }
}
