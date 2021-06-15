#include "software/ai/hl/stp/play/pass_endurance_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

PassEndurancePlay::PassEndurancePlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, false)
{
}

bool PassEndurancePlay::isApplicable(const World &world) const
{
    // This play is never applicable so it will never be chosen during gameplay
    // This play can be run for hardware challenges by using the Play override
    return false;
}

bool PassEndurancePlay::invariantHolds(const World &world) const
{
    return false;
}

void PassEndurancePlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, PassEndurancePlay, PlayConfig> factory;
