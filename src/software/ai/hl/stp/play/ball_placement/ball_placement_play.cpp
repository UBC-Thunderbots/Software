#include "software/ai/hl/stp/play/ball_placement/ball_placement_play.h"

#include "software/util/generic_factory/generic_factory.h"


BallPlacementPlay::BallPlacementPlay(TbotsProto::AiConfig config)
    : Play(config, true), fsm{BallPlacementPlayFSM{config}}, control_params{}
{
}

void BallPlacementPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    // This function doesn't get called so it does nothing, will be removed once
    // coroutines are phased out
}

void BallPlacementPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(BallPlacementPlayFSM::Update(control_params, play_update));
}

std::vector<std::string> BallPlacementPlay::getState()
{
    std::vector<std::string> state;
    state.emplace_back(objectTypeName(*this) + " - " + getCurrentFullStateName(fsm));
    return state;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, BallPlacementPlay, TbotsProto::AiConfig>
    factory;
