#include "software/ai/play_selection_fsm.h"

#include "software/ai/hl/stp/play/ball_placement/ball_placement_play.h"
#include "software/ai/hl/stp/play/corner_kick_play.h"
#include "software/ai/hl/stp/play/enemy_ball_placement/enemy_ball_placement_play.h"
#include "software/ai/hl/stp/play/enemy_free_kick/enemy_free_kick_play.h"
#include "software/ai/hl/stp/play/free_kick_play.h"
#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/play/kickoff_enemy_play.h"
#include "software/ai/hl/stp/play/kickoff_friendly_play.h"
#include "software/ai/hl/stp/play/offense/offense_play.h"
#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play.h"
#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play.h"
#include "software/ai/hl/stp/play/stop_play.h"


PlaySelectionFSM::PlaySelectionFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config), current_set_play(NONE)
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
        if (current_set_play != FRIENDLY_BALL_PLACEMENT)
        {
            current_set_play = FRIENDLY_BALL_PLACEMENT;
            event.set_current_play(std::make_unique<BallPlacementPlay>(ai_config));
        }
    }
    else if (event.game_state.isTheirBallPlacement())
    {
        if (current_set_play != ENEMY_BALL_PLACEMENT)
        {
            current_set_play = ENEMY_BALL_PLACEMENT;
            event.set_current_play(std::make_unique<EnemyBallPlacementPlay>(ai_config));
        }
    }
    else if (event.game_state.isOurKickoff())
    {
        if (current_set_play != FRIENDLY_KICKOFF)
        {
            current_set_play = FRIENDLY_KICKOFF;
            event.set_current_play(std::make_unique<KickoffFriendlyPlay>(ai_config));
        }
    }
    else if (event.game_state.isTheirKickoff())
    {
        if (current_set_play != ENEMY_KICKOFF)
        {
            current_set_play = ENEMY_KICKOFF;
            event.set_current_play(std::make_unique<KickoffEnemyPlay>(ai_config));
        }
    }
    else if (event.game_state.isOurPenalty())
    {
        if (current_set_play != FRIENDLY_PENALTY_KICK)
        {
            current_set_play = FRIENDLY_PENALTY_KICK;
            event.set_current_play(std::make_unique<PenaltyKickPlay>(ai_config));
        }
    }
    else if (event.game_state.isTheirPenalty())
    {
        if (current_set_play != ENEMY_PENALTY_KICK)
        {
            current_set_play = ENEMY_PENALTY_KICK;
            event.set_current_play(std::make_unique<PenaltyKickEnemyPlay>(ai_config));
        }
    }
    else if (event.game_state.isOurDirectFree() || event.game_state.isOurIndirectFree())
    {
        if (current_set_play != FRIENDLY_FREE_KICK)
        {
            current_set_play = FRIENDLY_FREE_KICK;
            event.set_current_play(std::make_unique<FreeKickPlay>(ai_config));
        }
    }
    else if (event.game_state.isTheirDirectFree() ||
             event.game_state.isTheirIndirectFree())
    {
        if (current_set_play != ENEMY_FREE_KICK)
        {
            current_set_play = ENEMY_FREE_KICK;
            event.set_current_play(std::make_unique<EnemyFreeKickPlay>(ai_config));
        }
    }
}

void PlaySelectionFSM::setupStopPlay(const Update& event)
{
    event.set_current_play(std::make_unique<StopPlay>(ai_config));
}

void PlaySelectionFSM::setupHaltPlay(const Update& event)
{
    event.set_current_play(std::make_unique<HaltPlay>(ai_config));
}

void PlaySelectionFSM::setupOffensePlay(const Update& event)
{
    event.set_current_play(std::make_unique<OffensePlay>(ai_config));
}

void PlaySelectionFSM::resetSetPlay(const Update& event)
{
    current_set_play = NONE;
}
