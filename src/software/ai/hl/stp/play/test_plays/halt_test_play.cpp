#include "software/ai/hl/stp/play/test_plays/halt_test_play.h"

#include "software/util/generic_factory/generic_factory.h"

HaltTestPlay::HaltTestPlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayBase<HaltTestPlayFSM>(ai_config_ptr, false)
{
}

void HaltTestPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                  const WorldPtr &world_ptr)
{
    // This function doesn't get called and will be removed when coroutines are phased
    // out.
}

void HaltTestPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(HaltTestPlayFSM::Update(control_params, play_update));
}
// Register this play in the genericFactory
static TGenericFactory<std::string, Play, HaltTestPlay,
                       std::shared_ptr<const TbotsProto::AiConfig>>
    factory;
