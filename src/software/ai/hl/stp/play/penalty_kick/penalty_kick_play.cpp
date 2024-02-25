#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/util/generic_factory/generic_factory.h"

PenaltyKickPlay::PenaltyKickPlay(std::shared_ptr<Strategy> strategy)
    : Play(true, strategy),
      fsm(std::make_unique<FSM<PenaltyKickPlayFSM>>(
          PenaltyKickPlayFSM(strategy->getAiConfig()))),
      control_params{}
{
}

void PenaltyKickPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                     const WorldPtr &world_ptr)
{
    // This function doesn't get called, it should be removed once coroutines are phased
    // out
}

void PenaltyKickPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm->process_event(PenaltyKickPlayFSM::Update(control_params, play_update));
}

std::vector<std::string> PenaltyKickPlay::getState()
{
    std::vector<std::string> state;
    state.emplace_back(objectTypeName(*this) + " - " + getCurrentFullStateName(*fsm));
    return state;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, PenaltyKickPlay, std::shared_ptr<Strategy>>
    factory;
