#include "software/world/game_state.h"

#include "software/logger/logger.h"

bool GameState::isHalted() const
{
    return state_ == HALT;
}

bool GameState::isStopped() const
{
    return state_ == STOP;
}

bool GameState::isPlaying() const
{
    return state_ == PLAYING;
}

bool GameState::isKickoff() const
{
    return restart_reason_ == KICKOFF;
}

bool GameState::isPenalty() const
{
    return restart_reason_ == PENALTY;
}

bool GameState::isBallPlacement() const
{
    return restart_reason_ == BALL_PLACEMENT;
}

bool GameState::isOurRestart() const
{
    // it has to be a restart for it to be our restart
    return our_restart_ && restart_reason_ != RestartReason::NONE;
}

bool GameState::isDirectFree() const
{
    return restart_reason_ == DIRECT;
}

bool GameState::isIndirectFree() const
{
    return restart_reason_ == INDIRECT;
}

bool GameState::isOurKickoff() const
{
    return isKickoff() && our_restart_;
}

bool GameState::isOurPenalty() const
{
    return isPenalty() && our_restart_;
}

bool GameState::isOurDirectFree() const
{
    return isDirectFree() && our_restart_;
}

bool GameState::isOurIndirectFree() const
{
    return isIndirectFree() && our_restart_;
}

bool GameState::isOurFreeKick() const
{
    return isOurDirectFree() || isOurIndirectFree();
}

bool GameState::isOurBallPlacement() const
{
    return isBallPlacement() && our_restart_;
}

bool GameState::isTheirKickoff() const
{
    return isKickoff() && !our_restart_;
}

bool GameState::isTheirPenalty() const
{
    return isPenalty() && !our_restart_;
}

bool GameState::isTheirDirectFree() const
{
    return isDirectFree() && !our_restart_;
}

bool GameState::isTheirIndirectFree() const
{
    return isIndirectFree() && !our_restart_;
}

bool GameState::isTheirFreeKick() const
{
    return isTheirDirectFree() || isTheirIndirectFree();
}

bool GameState::isTheirBallPlacement() const
{
    return isBallPlacement() && !our_restart_;
}

// Robots must be in position for a restart
bool GameState::isSetupRestart() const
{
    return state_ == SETUP || state_ == READY;
}

bool GameState::isSetupState() const
{
    return state_ == SETUP;
}

bool GameState::isReadyState() const
{
    return state_ == READY;
}

// One of our robots can kick the ball
bool GameState::canKick() const
{
    return state_ == PLAYING || (our_restart_ && state_ == READY);
}

bool GameState::stayAwayFromBall() const
{
    return state_ != PLAYING && !our_restart_;
}

// Our robots must stay on our half of the field
bool GameState::stayOnSide() const
{
    return isSetupRestart() && restart_reason_ == KICKOFF && !our_restart_;
}

// Our robots (except the penalty kicker) must stay a set distance behind the penalty line
bool GameState::stayBehindPenaltyLine() const
{
    return restart_reason_ == PENALTY;
}

void GameState::setBallPlacementPoint(Point placement_point)
{
    ball_placement_point_ = placement_point;
}

// apologies for this monster switch statement
void GameState::updateRefereeCommand(RefereeCommand game_state)
{
    if (game_state != game_state_)
    {
        game_state_ = game_state;

        switch (game_state)
        {
            case RefereeCommand::HALT:
                state_          = HALT;
                restart_reason_ = NONE;
                break;
            case RefereeCommand::STOP:
                state_          = STOP;
                restart_reason_ = NONE;
                our_restart_    = false;
                break;
            case RefereeCommand::NORMAL_START:
                state_ = READY;
                break;
            case RefereeCommand::FORCE_START:
                state_          = PLAYING;
                restart_reason_ = NONE;
                break;
            case RefereeCommand::PREPARE_KICKOFF_US:
                state_          = SETUP;
                restart_reason_ = KICKOFF;
                our_restart_    = true;
                break;
            case RefereeCommand::PREPARE_KICKOFF_THEM:
                state_          = SETUP;
                restart_reason_ = KICKOFF;
                our_restart_    = false;
                break;
            case RefereeCommand::PREPARE_PENALTY_US:
                state_          = SETUP;
                restart_reason_ = PENALTY;
                our_restart_    = true;
                break;
            case RefereeCommand::PREPARE_PENALTY_THEM:
                state_          = SETUP;
                restart_reason_ = PENALTY;
                our_restart_    = false;
                break;
            case RefereeCommand::DIRECT_FREE_US:
                state_          = READY;
                restart_reason_ = DIRECT;
                our_restart_    = true;
                break;
            case RefereeCommand::DIRECT_FREE_THEM:
                state_          = READY;
                restart_reason_ = DIRECT;
                our_restart_    = false;
                break;
            case RefereeCommand::INDIRECT_FREE_US:
                state_          = READY;
                restart_reason_ = INDIRECT;
                our_restart_    = true;
                break;
            case RefereeCommand::INDIRECT_FREE_THEM:
                state_          = READY;
                restart_reason_ = INDIRECT;
                our_restart_    = false;
                break;
            case RefereeCommand::TIMEOUT_US:
                state_          = HALT;
                restart_reason_ = NONE;
                break;
            case RefereeCommand::TIMEOUT_THEM:
                state_          = HALT;
                restart_reason_ = NONE;
                break;
            case RefereeCommand::GOAL_US:
                state_          = STOP;
                restart_reason_ = NONE;
                break;
            case RefereeCommand::GOAL_THEM:
                state_          = STOP;
                restart_reason_ = NONE;
                break;
            case RefereeCommand::BALL_PLACEMENT_US:
                state_          = SETUP;
                restart_reason_ = BALL_PLACEMENT;
                our_restart_    = true;
                break;
            case RefereeCommand::BALL_PLACEMENT_THEM:
                state_          = SETUP;
                restart_reason_ = BALL_PLACEMENT;
                our_restart_    = false;
                break;
            default:
                LOG(WARNING) << "Unrecognized RefereeCommand" << std::endl;
                break;
        }
    }
}

void GameState::updateBall(const Ball& ball)
{
    if (state_ == READY && restart_reason_ != PENALTY)
    {
        if (!ball_state_)
        {
            // Save the ball state_ so we can tell once it moves
            ball_state_ = ball;
        }
        else if ((ball.position() - ball_state_->position()).length() > 0.03)
        {
            // Once the ball has moved enough, the restart is finished
            setRestartCompleted();
            ball_state_ = std::nullopt;
        }
    }
}

GameState::RestartReason GameState::getRestartReason() const
{
    return restart_reason_;
}

const RefereeCommand& GameState::getRefereeCommand() const
{
    return game_state_;
}


void GameState::setRestartCompleted()
{
    state_          = PLAYING;
    restart_reason_ = NONE;
}

bool GameState::operator==(const GameState& other) const
{
    return this->state_ == other.state_ &&
           this->restart_reason_ == other.restart_reason_ &&
           this->game_state_ == other.game_state_ &&
           this->ball_state_ == other.ball_state_ &&
           this->our_restart_ == other.our_restart_ &&
           this->ball_placement_point_ == other.ball_placement_point_;
}

bool GameState::operator!=(const GameState& other) const
{
    return !(*this == other);
}
