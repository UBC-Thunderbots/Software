#include "software/world/game_state.h"

#include "software/logger/logger.h"

GameState::GameState(const TbotsProto::GameState& game_state_proto) : our_restart_(false)
{
    if (game_state_proto.has_ball())
    {
        ball_ = Ball(game_state_proto.ball());
    }
    else
    {
        ball_ = std::nullopt;
    }

    if (game_state_proto.has_ball_placement_point())
    {
        ball_placement_point_ = Point(game_state_proto.ball_placement_point().x_meters(),
                                      game_state_proto.ball_placement_point().y_meters());
    }
    else
    {
        ball_placement_point_ = std::nullopt;
    }

    switch (game_state_proto.play_state())
    {
        case TbotsProto::GameState_PlayState_PLAY_STATE_HALT:
            play_state_ = GameState::PlayState::HALT;
            break;
        case TbotsProto::GameState_PlayState_PLAY_STATE_STOP:
            play_state_ = GameState::PlayState::STOP;
            break;
        case TbotsProto::GameState_PlayState_PLAY_STATE_SETUP:
            play_state_ = GameState::PlayState::SETUP;
            break;
        case TbotsProto::GameState_PlayState_PLAY_STATE_READY:
            play_state_ = GameState::PlayState::READY;
            break;
        case TbotsProto::GameState_PlayState_PLAY_STATE_PLAYING:
            play_state_ = GameState::PlayState::PLAYING;
            break;
    }

    switch (game_state_proto.restart_reason())
    {
        case TbotsProto::GameState_RestartReason_RESTART_REASON_NONE:
            restart_reason_ = GameState::RestartReason::NONE;
            break;
        case TbotsProto::GameState_RestartReason_RESTART_REASON_KICKOFF:
            restart_reason_ = GameState::RestartReason::KICKOFF;
            break;
        case TbotsProto::GameState_RestartReason_RESTART_REASON_DIRECT:
            restart_reason_ = GameState::RestartReason::DIRECT;
            break;
        case TbotsProto::GameState_RestartReason_RESTART_REASON_INDIRECT:
            restart_reason_ = GameState::RestartReason::INDIRECT;
            break;
        case TbotsProto::GameState_RestartReason_RESTART_REASON_PENALTY:
            restart_reason_ = GameState::RestartReason::PENALTY;
            break;
        case TbotsProto::GameState_RestartReason_RESTART_REASON_BALL_PLACEMENT:
            restart_reason_ = GameState::RestartReason::BALL_PLACEMENT;
            break;
    }

    switch (game_state_proto.command())
    {
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_HALT:
            updateRefereeCommand(RefereeCommand::HALT);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_STOP:
            updateRefereeCommand(RefereeCommand::STOP);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_NORMAL_START:
            updateRefereeCommand(RefereeCommand::NORMAL_START);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_FORCE_START:
            updateRefereeCommand(RefereeCommand::FORCE_START);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_PREPARE_KICKOFF_US:
            updateRefereeCommand(RefereeCommand::PREPARE_KICKOFF_US);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_PREPARE_KICKOFF_THEM:
            updateRefereeCommand(RefereeCommand::PREPARE_KICKOFF_THEM);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_PREPARE_PENALTY_US:
            updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_PREPARE_PENALTY_THEM:
            updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_DIRECT_FREE_US:
            updateRefereeCommand(RefereeCommand::DIRECT_FREE_US);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_DIRECT_FREE_THEM:
            updateRefereeCommand(RefereeCommand::DIRECT_FREE_THEM);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_INDIRECT_FREE_US:
            updateRefereeCommand(RefereeCommand::INDIRECT_FREE_US);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_INDIRECT_FREE_THEM:
            updateRefereeCommand(RefereeCommand::INDIRECT_FREE_THEM);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_TIMEOUT_US:
            updateRefereeCommand(RefereeCommand::TIMEOUT_US);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_TIMEOUT_THEM:
            updateRefereeCommand(RefereeCommand::TIMEOUT_THEM);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_GOAL_US:
            updateRefereeCommand(RefereeCommand::GOAL_US);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_GOAL_THEM:
            updateRefereeCommand(RefereeCommand::GOAL_THEM);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_BALL_PLACEMENT_US:
            updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_US);
            break;
        case TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_BALL_PLACEMENT_THEM:
            updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_THEM);
            break;
    }
}

bool GameState::isHalted() const
{
    return play_state_ == HALT;
}

bool GameState::isStopped() const
{
    return play_state_ == STOP;
}

bool GameState::isPlaying() const
{
    return play_state_ == PLAYING;
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

GameState::PlayState GameState::getPlayState(void) const
{
    return play_state_;
}

// Robots must be in position for a restart
bool GameState::isSetupRestart() const
{
    return play_state_ == SETUP || play_state_ == READY;
}

bool GameState::isSetupState() const
{
    return play_state_ == SETUP;
}

bool GameState::isReadyState() const
{
    return play_state_ == READY;
}

bool GameState::stayAwayFromBall() const
{
    return play_state_ != PLAYING && !our_restart_;
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

std::optional<Point> GameState::getBallPlacementPoint() const
{
    return ball_placement_point_;
}

std::optional<Ball> GameState::getBall(void) const
{
    return ball_;
}

// apologies for this monster switch statement
void GameState::updateRefereeCommand(RefereeCommand command)
{
    if (command != command_)
    {
        command_ = command;

        switch (command)
        {
            case RefereeCommand::HALT:
                play_state_     = HALT;
                restart_reason_ = NONE;
                break;
            case RefereeCommand::STOP:
                play_state_     = STOP;
                restart_reason_ = NONE;
                our_restart_    = false;
                break;
            case RefereeCommand::NORMAL_START:
                play_state_ = READY;
                break;
            case RefereeCommand::FORCE_START:
                play_state_     = PLAYING;
                restart_reason_ = NONE;
                break;
            case RefereeCommand::PREPARE_KICKOFF_US:
                play_state_     = SETUP;
                restart_reason_ = KICKOFF;
                our_restart_    = true;
                break;
            case RefereeCommand::PREPARE_KICKOFF_THEM:
                play_state_     = SETUP;
                restart_reason_ = KICKOFF;
                our_restart_    = false;
                break;
            case RefereeCommand::PREPARE_PENALTY_US:
                play_state_     = SETUP;
                restart_reason_ = PENALTY;
                our_restart_    = true;
                break;
            case RefereeCommand::PREPARE_PENALTY_THEM:
                play_state_     = SETUP;
                restart_reason_ = PENALTY;
                our_restart_    = false;
                break;
            case RefereeCommand::DIRECT_FREE_US:
                play_state_     = SETUP;
                restart_reason_ = DIRECT;
                our_restart_    = true;
                break;
            case RefereeCommand::DIRECT_FREE_THEM:
                play_state_     = SETUP;
                restart_reason_ = DIRECT;
                our_restart_    = false;
                break;
            case RefereeCommand::INDIRECT_FREE_US:
                play_state_     = SETUP;
                restart_reason_ = INDIRECT;
                our_restart_    = true;
                break;
            case RefereeCommand::INDIRECT_FREE_THEM:
                play_state_     = SETUP;
                restart_reason_ = INDIRECT;
                our_restart_    = false;
                break;
            case RefereeCommand::TIMEOUT_US:
                play_state_     = HALT;
                restart_reason_ = NONE;
                break;
            case RefereeCommand::TIMEOUT_THEM:
                play_state_     = HALT;
                restart_reason_ = NONE;
                break;
            case RefereeCommand::GOAL_US:
                play_state_     = STOP;
                restart_reason_ = NONE;
                break;
            case RefereeCommand::GOAL_THEM:
                play_state_     = STOP;
                restart_reason_ = NONE;
                break;
            case RefereeCommand::BALL_PLACEMENT_US:
                play_state_     = SETUP;
                restart_reason_ = BALL_PLACEMENT;
                our_restart_    = true;
                break;
            case RefereeCommand::BALL_PLACEMENT_THEM:
                play_state_     = SETUP;
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
    if ((play_state_ == READY && restart_reason_ != PENALTY) ||
        restart_reason_ == DIRECT || restart_reason_ == INDIRECT)
    {
        if (!ball_)
        {
            // Save the ball play_state_ so we can tell once it moves
            ball_ = ball;
        }
        else if ((ball.position() - ball_->position()).length() > 0.03)
        {
            // Once the ball has moved enough, the restart is finished
            setRestartCompleted();
            ball_ = std::nullopt;
        }
    }
}

GameState::RestartReason GameState::getRestartReason() const
{
    return restart_reason_;
}

const RefereeCommand& GameState::getRefereeCommand() const
{
    return command_;
}


void GameState::setRestartCompleted()
{
    play_state_     = PLAYING;
    restart_reason_ = NONE;
}

bool GameState::operator==(const GameState& other) const
{
    return this->play_state_ == other.play_state_ &&
           this->restart_reason_ == other.restart_reason_ &&
           this->command_ == other.command_ && this->ball_ == other.ball_ &&
           this->our_restart_ == other.our_restart_ &&
           this->ball_placement_point_ == other.ball_placement_point_;
}

bool GameState::operator!=(const GameState& other) const
{
    return !(*this == other);
}
