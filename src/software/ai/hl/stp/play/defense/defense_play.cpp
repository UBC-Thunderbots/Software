#include "software/ai/hl/stp/play/defense/defense_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

DefensePlay::DefensePlay(const TbotsProto::AiConfig &config)
    : Play(config, true),
      fsm{DefensePlayFSM{config}},
      control_params{.max_allowed_speed_mode =
                         TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                     .defender_assignments = std::queue<DefenderAssignment>()}
{
}

void DefensePlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    // This function doesn't get called so it does nothing
    while (true)
    {
        yield({{}});
    }
}

void DefensePlay::updateControlParams(std::queue<DefenderAssignment> &defender_assignments, TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode)
{
    control_params.max_allowed_speed_mode = max_allowed_speed_mode;
    control_params.defender_assignments = defender_assignments;
}

void DefensePlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(DefensePlayFSM::Update(control_params, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, DefensePlay, TbotsProto::AiConfig> factory;
