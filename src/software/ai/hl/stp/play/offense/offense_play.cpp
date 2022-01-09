#include "software/ai/hl/stp/play/offense/offense_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/possession.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

using Zones = std::unordered_set<EighteenZoneId>;

OffensePlay::OffensePlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, true), fsm{OffensePlayFSM{config}}
{
}

bool OffensePlay::isApplicable(const World &world) const
{
    return world.gameState().isPlaying() &&
           (world.getTeamWithPossession() == TeamSide::FRIENDLY);
}

bool OffensePlay::invariantHolds(const World &world) const
{
    return world.gameState().isPlaying() &&
           (world.getTeamWithPossession() == TeamSide::FRIENDLY);
}

void OffensePlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    // This function doesn't get called so it does nothing
    while (true)
    {
        yield({{}});
    }
}

bool OffensePlay::done() const
{
    return fsm.is(boost::sml::X);
}

void OffensePlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(OffensePlayFSM::Update({}, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, OffensePlay, PlayConfig> factory;
