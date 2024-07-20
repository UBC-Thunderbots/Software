#include "software/ai/play_selection_fsm.h"

#include "software/ai/hl/stp/play/ball_placement/ball_placement_play.h"
#include "software/ai/hl/stp/play/enemy_ball_placement/enemy_ball_placement_play.h"
#include "software/ai/hl/stp/play/enemy_free_kick/enemy_free_kick_play.h"
#include "software/ai/hl/stp/play/free_kick/free_kick_play.h"
#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/play/kickoff_enemy_play.h"
#include "software/ai/hl/stp/play/kickoff_friendly_play.h"
#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play.h"
#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play.h"
#include "software/ai/hl/stp/play/stop_play.h"


PlaySelectionFSM::PlaySelectionFSM(std::shared_ptr<Strategy> strategy)
    : strategy_(strategy),
      offense_play_(std::make_shared<OffensePlay>(strategy)),
      current_set_play_(std::nullopt)
{
}

bool PlaySelectionFSM::gameStateStopped(const Update& event)
{
    return event.world_ptr->gameState().isStopped();
}

bool PlaySelectionFSM::gameStateHalted(const Update& event)
{
    return event.world_ptr->gameState().isHalted();
}

bool PlaySelectionFSM::gameStatePlaying(const Update& event)
{
    return event.world_ptr->gameState().isPlaying();
}

bool PlaySelectionFSM::gameStateSetupRestart(const Update& event)
{
    return event.world_ptr->gameState().isSetupRestart();
}

bool PlaySelectionFSM::enemyHasPossession(const Update& event)
{
    return event.world_ptr->getTeamWithPossession() == TeamPossession::ENEMY;
}

void PlaySelectionFSM::setupSetPlay(const Update& event)
{
    const GameState& game_state = event.world_ptr->gameState();

    if (game_state.isOurBallPlacement())
    {
        if (current_set_play_ != TbotsProto::PlayName::BallPlacementPlay)
        {
            current_set_play_ = TbotsProto::PlayName::BallPlacementPlay;
            event.set_current_play(std::make_unique<BallPlacementPlay>(strategy_));
        }
    }
    else if (game_state.isTheirBallPlacement())
    {
        if (current_set_play_ != TbotsProto::PlayName::EnemyBallPlacementPlay)
        {
            current_set_play_ = TbotsProto::PlayName::EnemyBallPlacementPlay;
            event.set_current_play(std::make_unique<EnemyBallPlacementPlay>(strategy_));
        }
    }
    else if (game_state.isOurKickoff())
    {
        if (current_set_play_ != TbotsProto::PlayName::KickoffFriendlyPlay)
        {
            current_set_play_ = TbotsProto::PlayName::KickoffFriendlyPlay;
            event.set_current_play(std::make_unique<KickoffFriendlyPlay>(strategy_));
        }
    }
    else if (game_state.isTheirKickoff())
    {
        if (current_set_play_ != TbotsProto::PlayName::KickoffEnemyPlay)
        {
            current_set_play_ = TbotsProto::PlayName::KickoffEnemyPlay;
            event.set_current_play(std::make_unique<KickoffEnemyPlay>(strategy_));
        }
    }
    else if (game_state.isOurPenalty())
    {
        if (current_set_play_ != TbotsProto::PlayName::PenaltyKickPlay)
        {
            current_set_play_ = TbotsProto::PlayName::PenaltyKickPlay;
            event.set_current_play(std::make_unique<PenaltyKickPlay>(strategy_));
        }
    }
    else if (game_state.isTheirPenalty())
    {
        if (current_set_play_ != TbotsProto::PlayName::PenaltyKickEnemyPlay)
        {
            current_set_play_ = TbotsProto::PlayName::PenaltyKickEnemyPlay;
            event.set_current_play(std::make_unique<PenaltyKickEnemyPlay>(strategy_));
        }
    }
    else if (game_state.isOurDirectFree() || game_state.isOurIndirectFree())
    {
        if (current_set_play_ != TbotsProto::PlayName::FreeKickPlay)
        {
            current_set_play_ = TbotsProto::PlayName::FreeKickPlay;
            event.set_current_play(std::make_unique<FreeKickPlay>(strategy_));
        }
    }
    else if (game_state.isTheirDirectFree() || game_state.isTheirIndirectFree())
    {
        if (current_set_play_ != TbotsProto::PlayName::EnemyFreeKickPlay)
        {
            current_set_play_ = TbotsProto::PlayName::EnemyFreeKickPlay;
            event.set_current_play(std::make_unique<EnemyFreeKickPlay>(strategy_));
        }
    }
}

void PlaySelectionFSM::setupStopPlay(const Update& event)
{
    event.set_current_play(std::make_shared<StopPlay>(strategy_));
}

void PlaySelectionFSM::setupHaltPlay(const Update& event)
{
    event.set_current_play(std::make_shared<HaltPlay>(strategy_));
}

void PlaySelectionFSM::setupOffensePlay(const Update& event)
{
    event.set_current_play(offense_play_);
}

void PlaySelectionFSM::setupDefensePlay(const Update& event)
{
    event.set_current_play(std::make_shared<DefensePlay>(strategy_));
}

void PlaySelectionFSM::terminateOffensePlay(const Update& event)
{
    offense_play_->terminate(event.world_ptr);
}

void PlaySelectionFSM::resetSetPlay(const Update& event)
{
    current_set_play_.reset();
}

bool PlaySelectionFSM::ballIsStagnant(const Update& event)
{
    Point current_ball_position = event.world_ptr->ball().position();
    const auto clock_time = std::chrono::system_clock::now();



    if (!last_ball_position.has_value() || distance(last_ball_position.value(), current_ball_position)
        > strategy_->getAiConfig().ai_parameter_config().ball_stagnant_distance_threshold_m())
    {
        // Reset the ball's possible stagnant location when it moves out of its threshold radius
        // and reset the initial time for a possible stagnant ball
        last_ball_position = current_ball_position;
        enemy_possession_epoch_time_s =
                static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                        clock_time.time_since_epoch())
                        .count()) *
                SECONDS_PER_MICROSECOND;
        return false;
    }
    double epoch_time_in_seconds =
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                    clock_time.time_since_epoch())
                    .count()) *
            SECONDS_PER_MICROSECOND;

    bool time_is_stagnant = epoch_time_in_seconds - enemy_possession_epoch_time_s
            >= strategy_->getAiConfig().ai_parameter_config().ball_stagnant_time_threshold_s();
    bool speed_is_stagnant = event.world_ptr->ball().velocity().length() <=
            strategy_->getAiConfig().ai_parameter_config().ball_stagnant_speed_threshold_m_per_s();

    return time_is_stagnant && speed_is_stagnant;
}
