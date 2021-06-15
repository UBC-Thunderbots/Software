#include "software/ai/hl/stp/play/scoring_with_static_defenders_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

ScoringWithStaticDefendersPlay::ScoringWithStaticDefendersPlay(
    std::shared_ptr<const PlayConfig> config)
    : Play(config, false)
{
}

bool ScoringWithStaticDefendersPlay::isApplicable(const World &world) const
{
    // This play is never applicable so it will never be chosen during gameplay
    // This play can be run for hardware challenges by using the Play override
    return false;
}

bool ScoringWithStaticDefendersPlay::invariantHolds(const World &world) const
{
    return false;
}

void ScoringWithStaticDefendersPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                                    const World &world)
{
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ScoringWithStaticDefendersPlay, PlayConfig>
    factory;
