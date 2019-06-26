#include "ai/hl/stp/play/halt_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/stop_tactic.h"

const std::string HaltPlay::name = "Halt Play";

std::string HaltPlay::getName() const
{
    return HaltPlay::name;
}

bool HaltPlay::isApplicable(const World &world) const
{
    return world.gameState().isHalted();
}

bool HaltPlay::invariantHolds(const World &world) const
{
    return world.gameState().isHalted();
}

void HaltPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // Create Stop Tactics that will loop forever
    auto stop_tactic_1 = std::make_shared<StopTactic>(false, true);
    auto stop_tactic_2 = std::make_shared<StopTactic>(false, true);
    auto stop_tactic_3 = std::make_shared<StopTactic>(false, true);
    auto stop_tactic_4 = std::make_shared<StopTactic>(false, true);
    auto stop_tactic_5 = std::make_shared<StopTactic>(false, true);
    auto stop_tactic_6 = std::make_shared<StopTactic>(false, true);

    do
    {
        stop_tactic_1->updateParams();
        stop_tactic_2->updateParams();
        stop_tactic_3->updateParams();
        stop_tactic_4->updateParams();
        stop_tactic_5->updateParams();
        stop_tactic_6->updateParams();

        // yield the Tactics this Play wants to run, in order of priority
        yield({stop_tactic_1, stop_tactic_2, stop_tactic_3, stop_tactic_4, stop_tactic_5,
               stop_tactic_6});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<HaltPlay> factory;
