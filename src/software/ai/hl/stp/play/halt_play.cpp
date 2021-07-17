#include "software/ai/hl/stp/play/halt_play.h"

#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

HaltPlay::HaltPlay(std::shared_ptr<const PlayConfig> config) : Play(config, false) {}

bool HaltPlay::isApplicable(const World &world) const
{
    return world.gameState().isHalted();
}

bool HaltPlay::invariantHolds(const World &world) const
{
    return world.gameState().isHalted();
}

void HaltPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    std::vector<std::shared_ptr<StopTactic>> stop_tactics(DIV_A_NUM_ROBOTS);
    std::generate(stop_tactics.begin(), stop_tactics.end(),
                  []() { return std::make_shared<StopTactic>(false); });

    do
    {
        TacticVector result = {};
        result.insert(result.end(), stop_tactics.begin(), stop_tactics.end());
        yield({result});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, HaltPlay, PlayConfig> factory;
