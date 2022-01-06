#include "software/ai/hl/stp/play/crease_defense/crease_defense_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

CreaseDefensePlay::CreaseDefensePlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, 1, true),
      fsm{CreaseDefensePlayFSM{config}},
      control_params{.enemy_threat_origin                     = Point(),
                     .num_additional_crease_defenders_tactics = 2,
                     .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT}
{
}

bool CreaseDefensePlay::isApplicable(const World &world) const
{
    return false;
}

bool CreaseDefensePlay::invariantHolds(const World &world) const
{
    return false;
}

void CreaseDefensePlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    // This function doesn't get called so it does nothing
    while (true)
    {
        yield({{}});
    }
}

bool CreaseDefensePlay::done() const
{
    return fsm.is(boost::sml::X);
}

void CreaseDefensePlay::updateControlParams(
    const Point &enemy_threat_origin,
    unsigned int num_additional_crease_defenders_tactics,
    MaxAllowedSpeedMode max_allowed_speed_mode)
{
    if (num_additional_crease_defenders_tactics > 2)
    {
        LOG(WARNING) << "CreaseDefensePlay can only handle 2 additional crease defenders"
                     << std::endl;
        num_additional_crease_defenders_tactics = 2;
    }
    control_params.enemy_threat_origin = enemy_threat_origin;
    control_params.num_additional_crease_defenders_tactics =
        num_additional_crease_defenders_tactics;
    control_params.max_allowed_speed_mode = max_allowed_speed_mode;
}

void CreaseDefensePlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(CreaseDefensePlayFSM::Update(control_params, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, CreaseDefensePlay, PlayConfig> factory;
