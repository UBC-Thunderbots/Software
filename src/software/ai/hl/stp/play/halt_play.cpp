#include "software/ai/hl/stp/play/halt_play.h"

#include "software/ai/hl/stp/tactic/stop_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

HaltPlay::HaltPlay(std::shared_ptr<const PlayConfig> config)
{
    play_config = config;
}

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
    // Create Stop Tactics that will loop forever
    auto stop_tactic_1 = std::make_shared<StopTactic>(false);
    auto stop_tactic_2 = std::make_shared<StopTactic>(false);
    auto stop_tactic_3 = std::make_shared<StopTactic>(false);
    auto stop_tactic_4 = std::make_shared<StopTactic>(false);
    auto stop_tactic_5 = std::make_shared<StopTactic>(false);
    auto stop_tactic_6 = std::make_shared<StopTactic>(false);

    do
    {
        // yield the Tactics this Play wants to run, in order of priority
        yield({stop_tactic_1, stop_tactic_2, stop_tactic_3, stop_tactic_4, stop_tactic_5,
               stop_tactic_6});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, HaltPlay, PlayConfig> factory;
