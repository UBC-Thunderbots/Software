#include "software/ai/hl/stp/play/defense/defense_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

DefensePlay::DefensePlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayBase<DefensePlayFSM>(ai_config_ptr, true)
{
}

void DefensePlay::getNextTactics(TacticCoroutine::push_type &yield,
                                 const WorldPtr &world_ptr)
{
    // This function doesn't get called so it does nothing
    while (true)
    {
        yield({{}});
    }
}

void DefensePlay::updateControlParams(
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode)
{
    control_params.max_allowed_speed_mode = max_allowed_speed_mode;
}

void DefensePlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(DefensePlayFSM::Update(control_params, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, DefensePlay,
                       std::shared_ptr<const TbotsProto::AiConfig>>
    factory;
