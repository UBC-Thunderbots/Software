#include "software/ai/play_selection_fsm.h"

#include "software/ai/hl/stp/play/ball_placement/ball_placement_play.h"
#include "software/ai/hl/stp/play/enemy_ball_placement/enemy_ball_placement_play.h"
#include "software/ai/hl/stp/play/enemy_free_kick/enemy_free_kick_play.h"
#include "software/ai/hl/stp/play/free_kick/free_kick_play.h"
#include "software/ai/hl/stp/play/halt_play/halt_play.h"
#include "software/ai/hl/stp/play/kickoff_enemy_play.h"
#include "software/ai/hl/stp/play/kickoff_friendly_play.h"
#include "software/ai/hl/stp/play/offense/offense_play.h"
#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play.h"
#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play.h"
#include "software/ai/hl/stp/play/stop_play.h"


PlaySelectionFSM::PlaySelectionFSM(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : ai_config_ptr(ai_config_ptr), current_set_play(std::nullopt)
{
}

bool PlaySelectionFSM::gameStateStopped(const Update& event)
{
    return event.game_state.isStopped();
}

bool PlaySelectionFSM::gameStateHalted(const Update& event)
{
    return event.game_state.isHalted();
}

bool PlaySelectionFSM::gameStatePlaying(const Update& event)
{
    return event.game_state.isPlaying();
}

bool PlaySelectionFSM::gameStateSetupRestart(const Update& event)
{
    return event.game_state.isSetupRestart();
}

void PlaySelectionFSM::setupSetPlay(const Update& event)
{
    if (event.game_state.isOurBallPlacement())
    {
        if (current_set_play != TbotsProto::PlayName::BallPlacementPlay)
        {
            current_set_play = TbotsProto::PlayName::BallPlacementPlay;
            event.set_current_play(std::make_unique<BallPlacementPlay>(ai_config_ptr));
        }
    }
    else if (event.game_state.isTheirBallPlacement())
    {
        if (current_set_play != TbotsProto::PlayName::EnemyBallPlacementPlay)
        {
            current_set_play = TbotsProto::PlayName::EnemyBallPlacementPlay;
            event.set_current_play(
                std::make_unique<EnemyBallPlacementPlay>(ai_config_ptr));
        }
    }
    else if (event.game_state.isOurKickoff())
    {
        if (current_set_play != TbotsProto::PlayName::KickoffFriendlyPlay)
        {
            current_set_play = TbotsProto::PlayName::KickoffFriendlyPlay;
            event.set_current_play(std::make_unique<KickoffFriendlyPlay>(ai_config_ptr));
        }
    }
    else if (event.game_state.isTheirKickoff())
    {
        if (current_set_play != TbotsProto::PlayName::KickoffEnemyPlay)
        {
            current_set_play = TbotsProto::PlayName::KickoffEnemyPlay;
            event.set_current_play(std::make_unique<KickoffEnemyPlay>(ai_config_ptr));
        }
    }
    else if (event.game_state.isOurPenalty())
    {
        if (current_set_play != TbotsProto::PlayName::PenaltyKickPlay)
        {
            current_set_play = TbotsProto::PlayName::PenaltyKickPlay;
            event.set_current_play(std::make_unique<PenaltyKickPlay>(ai_config_ptr));
        }
    }
    else if (event.game_state.isTheirPenalty())
    {
        if (current_set_play != TbotsProto::PlayName::PenaltyKickEnemyPlay)
        {
            current_set_play = TbotsProto::PlayName::PenaltyKickEnemyPlay;
            event.set_current_play(std::make_unique<PenaltyKickEnemyPlay>(ai_config_ptr));
        }
    }
    else if (event.game_state.isOurDirectFree() || event.game_state.isOurIndirectFree())
    {
        if (current_set_play != TbotsProto::PlayName::FreeKickPlay)
        {
            current_set_play = TbotsProto::PlayName::FreeKickPlay;
            event.set_current_play(std::make_unique<FreeKickPlay>(ai_config_ptr));
        }
    }
    else if (event.game_state.isTheirDirectFree() ||
             event.game_state.isTheirIndirectFree())
    {
        if (current_set_play != TbotsProto::PlayName::EnemyFreeKickPlay)
        {
            current_set_play = TbotsProto::PlayName::EnemyFreeKickPlay;
            event.set_current_play(std::make_unique<EnemyFreeKickPlay>(ai_config_ptr));
        }
    }
}

void PlaySelectionFSM::setupStopPlay(const Update& event)
{
    event.set_current_play(std::make_unique<StopPlay>(ai_config_ptr));
}

void PlaySelectionFSM::setupHaltPlay(const Update& event)
{
    event.set_current_play(std::make_unique<HaltPlay>(ai_config_ptr));
}

void PlaySelectionFSM::setupOffensePlay(const Update& event)
{
    event.set_current_play(std::make_unique<OffensePlay>(ai_config_ptr));
}

void PlaySelectionFSM::resetSetPlay(const Update& event)
{
    current_set_play.reset();
}
