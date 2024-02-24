#include "software/ai/hl/stp/play/enemy_ball_placement/enemy_ball_placement_play.h"

#include "software/util/generic_factory/generic_factory.h"

EnemyBallPlacementPlay::EnemyBallPlacementPlay(TbotsProto::AiConfig config)
    : Play(config, true), fsm{EnemyBallPlacementPlayFSM{config}}, control_params{}
{
}

void EnemyBallPlacementPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                            const WorldPtr &world_ptr)
{
    // This function doesn't get called so it does nothing, will be removed once
    // coroutines are phased out
}

void EnemyBallPlacementPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(EnemyBallPlacementPlayFSM::Update(control_params, play_update));
}

std::vector<std::string> EnemyBallPlacementPlay::getState()
{
    std::vector<std::string> state;
    state.emplace_back(objectTypeName(*this) + " - " + getCurrentFullStateName(fsm));
    return state;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, EnemyBallPlacementPlay, TbotsProto::AiConfig>
    factory;
