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


PlaySelectionFSM::PlaySelectionFSM(const TbotsProto::AiConfig& ai_config,
                                   std::shared_ptr<Strategy> strategy)
    : ai_config(ai_config),
      ball_placement_play(std::make_shared<BallPlacementPlay>(ai_config, strategy)),
      enemy_ball_placement_play(
          std::make_shared<EnemyBallPlacementPlay>(ai_config, strategy)),
      enemy_free_kick_play(std::make_shared<EnemyFreekickPlay>(ai_config, strategy)),
      free_kick_play(std::make_shared<FreeKickPlay>(ai_config, strategy)),
      halt_play(std::make_shared<HaltPlay>(ai_config, strategy)),
      kickoff_enemy_play(std::make_shared<KickoffEnemyPlay>(ai_config, strategy)),
      kickoff_friendly_play(std::make_shared<KickoffFriendlyPlay>(ai_config, strategy)),
      offense_play(std::make_shared<OffensePlay>(ai_config, strategy)),
      penalty_kick_enemy_play(
          std::make_shared<PenaltyKickEnemyPlay>(ai_config, strategy)),
      penalty_kick_play(std::make_shared<PenaltyKickPlay>(ai_config, strategy)),
      stop_play(std::make_shared<StopPlay>(ai_config, strategy))
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
