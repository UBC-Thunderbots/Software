#include "ai/world/game_state.h"

#include "game_state.h"


bool GameState::halt() const
{
    return state == HALT;
}

bool GameState::stopped() const
{
    return state == STOP;
}

bool GameState::playing() const
{
    return state == PLAYING;
}

bool GameState::kickoff() const
{
    return restart == KICKOFF;
}

bool GameState::penalty() const
{
    return restart == PENALTY;
}

bool GameState::ballPlacement() const
{
    return restart == BALL_PLACEMENT;
}

bool GameState::ourRestart() const
{
    return our_restart;
}

bool GameState::direct() const
{
    return restart == DIRECT;
}

bool GameState::indirect() const
{
    return restart == INDIRECT;
}

bool GameState::ourKickoff() const
{
    return kickoff() && our_restart;
}

bool GameState::ourPenalty() const
{
    return penalty() && our_restart;
}

bool GameState::ourDirect() const
{
    return direct() && our_restart;
}

bool GameState::ourIndirect() const
{
    return indirect() && our_restart;
}

bool GameState::ourFreeKick() const
{
    return ourDirect() || ourIndirect();
}

bool GameState::ourPlacement() const
{
    return ballPlacement() && our_restart;
}

bool GameState::theirKickoff() const
{
    return kickoff() && !our_restart;
}

bool GameState::theirPenalty() const
{
    return penalty() && !our_restart;
}

bool GameState::theirDirect() const
{
    return direct() && !our_restart;
}

bool GameState::theirIndirect() const
{
    return indirect() && !our_restart;
}

bool GameState::theirFreeKick() const
{
    return theirDirect() || theirIndirect();
}

bool GameState::theirPlacement() const
{
    return ballPlacement() && !our_restart;
}

// Robots must be in position for a restart
bool GameState::setupRestart() const
{
    return state == SETUP || state == READY;
}

bool GameState::setupState() const
{
    return state == SETUP;
}

bool GameState::readyState() const
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
    return setupRestart() && restart == KICKOFF;
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
