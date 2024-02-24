#include "software/ai/play_selection_fsm.h"

#include "software/ai/hl/stp/play/ball_placement_play.h"
#include "software/ai/hl/stp/play/corner_kick_play.h"
#include "software/ai/hl/stp/play/enemy_ball_placement_play.h"
#include "software/ai/hl/stp/play/enemy_free_kick_play.h"
#include "software/ai/hl/stp/play/free_kick_play.h"
#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/play/kickoff_enemy_play.h"
#include "software/ai/hl/stp/play/kickoff_friendly_play.h"
#include "software/ai/hl/stp/play/offense/offense_play.h"
#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play.h"
#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play.h"
#include "software/ai/hl/stp/play/stop_play.h"


PlaySelectionFSM::PlaySelectionFSM(std::shared_ptr<Strategy> strategy)
    : ai_config(strategy->getAiConfig()),
      ball_placement_play(std::make_shared<BallPlacementPlay>(strategy)),
      enemy_ball_placement_play(
          std::make_shared<EnemyBallPlacementPlay>(strategy)),
      enemy_free_kick_play(std::make_shared<EnemyFreekickPlay>(strategy)),
      free_kick_play(std::make_shared<FreeKickPlay>(strategy)),
      halt_play(std::make_shared<HaltPlay>(strategy)),
      kickoff_enemy_play(std::make_shared<KickoffEnemyPlay>(strategy)),
      kickoff_friendly_play(std::make_shared<KickoffFriendlyPlay>(strategy)),
      offense_play(std::make_shared<OffensePlay>(strategy)),
      penalty_kick_enemy_play(
          std::make_shared<PenaltyKickEnemyPlay>(strategy)),
      penalty_kick_play(std::make_shared<PenaltyKickPlay>(strategy)),
      stop_play(std::make_shared<StopPlay>(strategy))
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
        event.set_current_play(ball_placement_play);
    }

    if (event.game_state.isTheirBallPlacement())
    {
        event.set_current_play(enemy_ball_placement_play);
    }

    if (event.game_state.isOurKickoff())
    {
        event.set_current_play(kickoff_friendly_play);
    }

    if (event.game_state.isTheirKickoff())
    {
        event.set_current_play(kickoff_enemy_play);
    }

    if (event.game_state.isOurPenalty())
    {
        event.set_current_play(penalty_kick_play);
    }

    if (event.game_state.isTheirPenalty())
    {
        event.set_current_play(penalty_kick_enemy_play);
    }

    if (event.game_state.isOurDirectFree() || event.game_state.isOurIndirectFree())
    {
        event.set_current_play(free_kick_play);
    }

    if (event.game_state.isTheirDirectFree() || event.game_state.isTheirIndirectFree())
    {
        event.set_current_play(enemy_free_kick_play);
    }
}

void PlaySelectionFSM::setupStopPlay(const Update& event)
{
    event.set_current_play(stop_play);
}

void PlaySelectionFSM::setupHaltPlay(const Update& event)
{
    event.set_current_play(halt_play);
}

void PlaySelectionFSM::setupOffensePlay(const Update& event)
{
    event.set_current_play(offense_play);
}
