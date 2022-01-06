#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

using Zones = std::unordered_set<EighteenZoneId>;

ShootOrPassPlay::ShootOrPassPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, 2, true),
      fsm{ShootOrPassPlayFSM{config}},
      control_params{.num_additional_offensive_tactics = 3}
{
}

bool ShootOrPassPlay::isApplicable(const World &world) const
{
    return false;
}

bool ShootOrPassPlay::invariantHolds(const World &world) const
{
    return false;
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

bool ShootOrPassPlay::done() const
{
    return fsm.is(boost::sml::X);
}

void ShootOrPassPlay::updateControlParams(unsigned int num_additional_offensive_tactics)
{
    control_params.num_additional_offensive_tactics = num_additional_offensive_tactics;
}

void ShootOrPassPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(ShootOrPassPlayFSM::Update(control_params, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ShootOrPassPlay, PlayConfig> factory;
