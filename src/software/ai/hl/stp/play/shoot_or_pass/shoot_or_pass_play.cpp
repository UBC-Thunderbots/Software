#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

ShootOrPassPlay::ShootOrPassPlay(const TbotsProto::AiConfig &config,
                                 std::shared_ptr<Strategy> strategy)
    : Play(config, true, strategy),
      fsm(std::make_unique<FSM<ShootOrPassPlayFSM>>(
          ShootOrPassPlayFSM(config, strategy))),
      control_params{}
{
}

void ShootOrPassPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                     const World &world)
{
    // This function doesn't get called so it does nothing
    while (true)
    {
        yield({{}});
    }
}

void ShootOrPassPlay::reset()
{
    Play::reset();

    fsm = std::make_unique<FSM<ShootOrPassPlayFSM>>(
        ShootOrPassPlayFSM(ai_config, strategy));
}

void ShootOrPassPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm->process_event(ShootOrPassPlayFSM::Update(control_params, play_update));
}

std::vector<std::string> ShootOrPassPlay::getState()
{
    std::vector<std::string> state;
    state.emplace_back(objectTypeName(*this) + " - " + getCurrentFullStateName(*fsm));
    return state;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ShootOrPassPlay, TbotsProto::AiConfig,
                       std::shared_ptr<Strategy>>
    factory;
