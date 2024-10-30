#include "software/ai/hl/stp/play/example/example_play.h"

#include "shared/constants.h"
#include "software/util/generic_factory/generic_factory.h"

void ExamplePlay::getNextTactics(TacticCoroutine::push_type &yield,
                                 const WorldPtr &world_ptr)
{
    // This function doesn't get called and it will be removed once coroutines are phased
    // out
}

void ExamplePlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(ExamplePlayFSM::Update(control_params, play_update));
}

std::vector<std::string> ExamplePlay::getState()
{
    std::vector<std::string> state;
    state.emplace_back(objectTypeName(*this) + " - " + getCurrentFullStateName(fsm));
    return state;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ExamplePlay, TbotsProto::AiConfig> factory;
