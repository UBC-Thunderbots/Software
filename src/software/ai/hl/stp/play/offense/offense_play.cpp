#include "software/ai/hl/stp/play/offense/offense_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/util/generic_factory/generic_factory.h"

OffensePlay::OffensePlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayBase<OffensePlayFSM>(ai_config_ptr, true)
{
}

void OffensePlay::getNextTactics(TacticCoroutine::push_type &yield,
                                 const WorldPtr &world_ptr)
{
    // This function doesn't get called, it should be removed once coroutines
    // are phased out
    while (true)
    {
        yield({{}});
    }
}

void OffensePlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(OffensePlayFSM::Update(control_params, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, OffensePlay,
                       std::shared_ptr<const TbotsProto::AiConfig>>
    factory;
