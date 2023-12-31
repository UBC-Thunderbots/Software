#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play.h"

#include "shared/constants.h"
#include "software/util/generic_factory/generic_factory.h"

PenaltyKickEnemyPlay::PenaltyKickEnemyPlay(const TbotsProto::AiConfig& config, std::shared_ptr<Strategy> strategy)
    : Play(config, true, strategy),
      control_params{.goalie_tactic = goalie_tactic}
{
}

void PenaltyKickEnemyPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                          const World &world)
{
    // This function doesn't get called, it should be removed once coroutines
    // are phased out
}

void PenaltyKickEnemyPlay::reset(const TbotsProto::AiConfig& config)
{
    Play::reset(config);

    fsm = std::make_unique<FSM<PenaltyKickEnemyPlayFSM>>(PenaltyKickEnemyPlayFSM(config));
}

void PenaltyKickEnemyPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm->process_event(PenaltyKickEnemyPlayFSM::Update(control_params, play_update));
}

std::vector<std::string> PenaltyKickEnemyPlay::getState()
{
    std::vector<std::string> state;
    state.emplace_back(objectTypeName(*this) + " - " + getCurrentFullStateName(*fsm));
    return state;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, PenaltyKickEnemyPlay, TbotsProto::AiConfig, std::shared_ptr<Strategy>>
    factory;
