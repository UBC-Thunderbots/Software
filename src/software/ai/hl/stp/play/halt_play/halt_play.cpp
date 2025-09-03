#include "software/ai/hl/stp/play/halt_play/halt_play.h"

#include "software/ai/hl/stp/tactic/halt/halt_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

HaltPlay::HaltPlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayBase<HaltPlayFSM>(ai_config_ptr, false)
{
}

void HaltPlay::getNextTactics(TacticCoroutine::push_type &yield,
                              const WorldPtr &world_ptr)
{
    // This function doesn't get called and will be removed when coroutines are phased
    // out.
}

void HaltPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(HaltPlayFSM::Update(control_params, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, HaltPlay, std::shared_ptr<const TbotsProto::AiConfig>>
    factory;
