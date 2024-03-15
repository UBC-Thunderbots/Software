#include "software/ai/play_selection_fsm.h"

#include "software/ai/hl/stp/play/ball_placement_play.h"
#include "software/ai/hl/stp/play/enemy_ball_placement_play.h"
#include "software/ai/hl/stp/play/enemy_free_kick_play.h"
#include "software/ai/hl/stp/play/free_kick_play.h"
#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/play/kickoff_enemy_play.h"
#include "software/ai/hl/stp/play/kickoff_friendly_play.h"
#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play.h"
#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play.h"
#include "software/ai/hl/stp/play/stop_play.h"


PlaySelectionFSM::PlaySelectionFSM(std::shared_ptr<Strategy> strategy)
    : current_dynamic_play_(nullptr),
      offensive_friendly_third_play_(
          std::make_shared<OffensiveFriendlyThirdPlay>(strategy)),
      offensive_middle_third_play_(std::make_shared<OffensiveMiddleThirdPlay>(strategy)),
      offensive_enemy_third_play_(std::make_shared<OffensiveEnemyThirdPlay>(strategy)),
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
    return event.world_ptr->getTeamWithPossession() == TeamPossession::ENEMY_TEAM;
}

void PlaySelectionFSM::setupSetPlay(const Update& event)
{
    const GameState& game_state = event.world_ptr->gameState();

    if (game_state.isOurBallPlacement())
    {
        event.set_current_play(std::make_shared<BallPlacementPlay>(strategy));
    }

    if (game_state.isTheirBallPlacement())
    {
        event.set_current_play(std::make_shared<EnemyBallPlacementPlay>(strategy));
    }

    if (game_state.isOurKickoff())
    {
        event.set_current_play(std::make_shared<KickoffFriendlyPlay>(strategy));
    }

    if (game_state.isTheirKickoff())
    {
        event.set_current_play(std::make_shared<KickoffEnemyPlay>(strategy));
    }

    if (game_state.isOurPenalty())
    {
        event.set_current_play(std::make_shared<PenaltyKickPlay>(strategy));
    }

    if (game_state.isTheirPenalty())
    {
        event.set_current_play(std::make_shared<PenaltyKickEnemyPlay>(strategy));
    }

    if (game_state.isOurDirectFree() || game_state.isOurIndirectFree())
    {
        event.set_current_play(std::make_shared<FreeKickPlay>(strategy));
    }

    if (game_state.isTheirDirectFree() || game_state.isTheirIndirectFree())
    {
        event.set_current_play(std::make_shared<EnemyFreekickPlay>(strategy));
    }
}

void PlaySelectionFSM::setupStopPlay(const Update& event)
{
    event.set_current_play(std::make_shared<StopPlay>(strategy));
}

void PlaySelectionFSM::setupHaltPlay(const Update& event)
{
    event.set_current_play(std::make_shared<HaltPlay>(strategy));
}

void PlaySelectionFSM::setupOffensivePlay(const Update& event)
{
    const Field& field = event.world_ptr->field();
    const double third = field.xLength() / 3;
    Rectangle friendly_third(
        Point(field.friendlyCornerPos().x() + third, field.friendlyCornerPos().y()),
        field.friendlyCornerNeg());
    Rectangle enemy_third(
        Point(field.enemyCornerPos().x() - third, field.enemyCornerPos().y()),
        field.enemyCornerNeg());

    Point ball_position = event.world_ptr->ball().position();

    if (contains(friendly_third, ball_position))
    {
        current_dynamic_play_ = offensive_friendly_third_play_;
    }
    else if (contains(enemy_third, ball_position))
    {
        current_dynamic_play_ = offensive_enemy_third_play_;
    }
    else
    {
        current_dynamic_play_ = offensive_middle_third_play_;
    }

    event.set_current_play(current_dynamic_play_);
}

void PlaySelectionFSM::setupDefensivePlay(const Update& event)
{
    event.set_current_play(std::make_shared<DefensePlay>(strategy));
}

void PlaySelectionFSM::evaluateDynamicPlay(const Update& event)
{
    if (current_dynamic_play_)
    {
        current_dynamic_play_->evaluate();
    }
}