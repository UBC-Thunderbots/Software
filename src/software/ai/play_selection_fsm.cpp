#include "software/ai/play_selection_fsm.h"

#include "software/ai/hl/stp/play/ball_placement/ball_placement_play.h"
#include "software/ai/hl/stp/play/enemy_ball_placement/enemy_ball_placement_play.h"
#include "software/ai/hl/stp/play/enemy_free_kick/enemy_free_kick_play.h"
#include "software/ai/hl/stp/play/free_kick_play.h"
#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/play/kickoff_enemy_play.h"
#include "software/ai/hl/stp/play/kickoff_friendly_play.h"
#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play.h"
#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play.h"
#include "software/ai/hl/stp/play/stop_play.h"


PlaySelectionFSM::PlaySelectionFSM(std::shared_ptr<Strategy> strategy)
    : strategy_(strategy),
      offense_play_(std::make_shared<OffensePlay>(strategy)),
      current_set_play_(NONE)
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
        if (current_set_play_ != FRIENDLY_BALL_PLACEMENT)
        {
            current_set_play_ = FRIENDLY_BALL_PLACEMENT;
            event.set_current_play(std::make_unique<BallPlacementPlay>(strategy_));
        }
    }
    else if (game_state.isTheirBallPlacement())
    {
        if (current_set_play_ != ENEMY_BALL_PLACEMENT)
        {
            current_set_play_ = ENEMY_BALL_PLACEMENT;
            event.set_current_play(std::make_unique<EnemyBallPlacementPlay>(strategy_));
        }
    }
    else if (game_state.isOurKickoff())
    {
        if (current_set_play_ != FRIENDLY_KICKOFF)
        {
            current_set_play_ = FRIENDLY_KICKOFF;
            event.set_current_play(std::make_unique<KickoffFriendlyPlay>(strategy_));
        }
    }
    else if (game_state.isTheirKickoff())
    {
        if (current_set_play_ != ENEMY_KICKOFF)
        {
            current_set_play_ = ENEMY_KICKOFF;
            event.set_current_play(std::make_unique<KickoffEnemyPlay>(strategy_));
        }
    }
    else if (game_state.isOurPenalty())
    {
        if (current_set_play_ != FRIENDLY_PENALTY_KICK)
        {
            current_set_play_ = FRIENDLY_PENALTY_KICK;
            event.set_current_play(std::make_unique<PenaltyKickPlay>(strategy_));
        }
    }
    else if (game_state.isTheirPenalty())
    {
        if (current_set_play_ != ENEMY_PENALTY_KICK)
        {
            current_set_play_ = ENEMY_PENALTY_KICK;
            event.set_current_play(std::make_unique<PenaltyKickEnemyPlay>(strategy_));
        }
    }
    else if (game_state.isOurDirectFree() || game_state.isOurIndirectFree())
    {
        if (current_set_play_ != FRIENDLY_FREE_KICK)
        {
            current_set_play_ = FRIENDLY_FREE_KICK;
            event.set_current_play(std::make_unique<FreeKickPlay>(strategy_));
        }
    }
    else if (game_state.isTheirDirectFree() || game_state.isTheirIndirectFree())
    {
        if (current_set_play_ != ENEMY_FREE_KICK)
        {
            current_set_play_ = ENEMY_FREE_KICK;
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
    current_set_play_ = NONE;
}
