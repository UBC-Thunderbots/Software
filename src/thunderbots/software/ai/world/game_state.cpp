#include "ai/world/game_state.h"

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

bool GameState::isOurRestart() const
{
    return ourRestart;
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
    return kickoff() && ourRestart;
}

bool GameState::ourPenalty() const
{
    return penalty() && ourRestart;
}

bool GameState::ourDirect() const
{
    return direct() && ourRestart;
}

bool GameState::ourIndirect() const
{
    return indirect() && ourRestart;
}

bool GameState::ourFreeKick() const
{
    return ourDirect() || ourIndirect();
}

bool GameState::ourPlacement() const
{
    return ballPlacement() && ourRestart;
}

bool GameState::theirKickoff() const
{
    return kickoff() && !ourRestart;
}

bool GameState::theirPenalty() const
{
    return penalty() && !ourRestart;
}

bool GameState::theirDirect() const
{
    return direct() && !ourRestart;
}

bool GameState::theirIndirect() const
{
    return indirect() && !ourRestart;
}

bool GameState::theirFreeKick() const
{
    return theirDirect() || theirIndirect();
}

bool GameState::theirPlacement() const
{
    return ballPlacement() && !ourRestart;
}

// Robots must be in position for a restart
bool GameState::setupRestart() const
{
    return state == SETUP || state == READY;
}

bool GameState::inSetupState() const
{
    return state == SETUP;
}

bool GameState::inReadyState() const
{
    return state == READY;
}

// One of our robots can kick the ball
bool GameState::canKick() const
{
    return state == PLAYING || (ourRestart && state == PLAYING);
}

// Our robots must stay 500mm away from the ball
bool GameState::stayAwayFromBall() const
{
    return state != PLAYING && !ourRestart;
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

void GameState::setBallPlacementPoint(double x, double y)
{
    // TODO: coordinate transform?
}

Point GameState::getBallPlacementPoint() const
{
    return ballPlacementPoint;
}