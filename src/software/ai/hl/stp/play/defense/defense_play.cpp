#include "software/ai/hl/stp/play/defense/defense_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

DefensePlay::DefensePlay(const TbotsProto::AiConfig &config,
                         std::shared_ptr<Strategy> strategy)
    : Play(config, true, strategy),
      fsm(std::make_unique<FSM<DefensePlayFSM>>(DefensePlayFSM(config))),
      control_params{.max_allowed_speed_mode =
                         TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT}
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

void DefensePlay::reset()
{
    Play::reset();

    fsm = std::make_unique<FSM<DefensePlayFSM>>(DefensePlayFSM(ai_config));
}

void DefensePlay::updateControlParams(
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode)
{
    control_params.max_allowed_speed_mode = max_allowed_speed_mode;
}

void DefensePlay::updateTactics(const PlayUpdate &play_update)
{
    fsm->process_event(DefensePlayFSM::Update(control_params, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, DefensePlay, TbotsProto::AiConfig,
                       std::shared_ptr<Strategy>>
    factory;
