#include "ai/hl/stp/play/idle_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/idle_tactic.h"

const std::string IdlePlay::name = "Idle Play";

std::string IdlePlay::getName() const
{
    return IdlePlay::name;
}

bool IdlePlay::isApplicable(const World &world) const
{
    return true;
}

bool IdlePlay::invariantHolds(const World &world) const
{
    return true;
}

void IdlePlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // Create Idle Tactics that will loop forever
    auto idle_tactic_1 = std::make_shared<IdleTactic>(true);
    auto idle_tactic_2 = std::make_shared<IdleTactic>(true);
    auto idle_tactic_3 = std::make_shared<IdleTactic>(true);
    auto idle_tactic_4 = std::make_shared<IdleTactic>(true);
    auto idle_tactic_5 = std::make_shared<IdleTactic>(true);
    auto idle_tactic_6 = std::make_shared<IdleTactic>(true);

    do
    {
        idle_tactic_1->updateParams();
        idle_tactic_2->updateParams();
        idle_tactic_3->updateParams();
        idle_tactic_4->updateParams();
        idle_tactic_5->updateParams();
        idle_tactic_6->updateParams();

        // yield the Tactics this Play wants to run, in order of priority
        yield({idle_tactic_1, idle_tactic_2, idle_tactic_3, idle_tactic_4, idle_tactic_5,
               idle_tactic_6});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<IdlePlay> factory;
