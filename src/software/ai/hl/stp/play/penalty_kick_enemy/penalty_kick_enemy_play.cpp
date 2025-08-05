#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play.h"

#include "shared/constants.h"
#include "software/util/generic_factory/generic_factory.h"

PenaltyKickEnemyPlay::PenaltyKickEnemyPlay(
    std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : PlayBase<PenaltyKickEnemyPlayFSM>(ai_config_ptr, true)
{
    // Assign the shared pointer to goalie tactic in the Play superclass to the control params instance.
    // This must be done after construction of every superclass, since control_params and goalie_tactic are defined in PlayBase and Play respectively.
    control_params.goalie_tactic = goalie_tactic;
}

void PenaltyKickEnemyPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                          const WorldPtr &world_ptr)
{
    // This function doesn't get called, it should be removed once coroutines
    // are phased out
}

void PenaltyKickEnemyPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(PenaltyKickEnemyPlayFSM::Update(control_params, play_update));
}

std::vector<std::string> PenaltyKickEnemyPlay::getState()
{
    std::vector<std::string> state;
    state.emplace_back(objectTypeName(*this) + " - " + getCurrentFullStateName(fsm));
    return state;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, PenaltyKickEnemyPlay,
                       std::shared_ptr<TbotsProto::AiConfig>>
    factory;
