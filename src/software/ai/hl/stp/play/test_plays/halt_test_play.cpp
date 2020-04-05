#include "software/ai/hl/stp/play/test_plays/halt_test_play.h"

#include "software/ai/hl/stp/tactic/test_tactics/stop_test_tactic.h"
#include "software/geom/util.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string HaltTestPlay::name = "Halt Test Play";

std::string HaltTestPlay::getName() const
{
    return HaltTestPlay::name;
}

bool HaltTestPlay::isApplicable(const World &world) const
{
    return world.ball().position().y() >= 0;
}

bool HaltTestPlay::invariantHolds(const World &world) const
{
    return Rectangle(world.field().enemyCornerNeg(), world.field().friendlyCornerPos())
        .contains(world.ball().position());
}

void HaltTestPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    auto stop_test_tactic_1 = std::make_shared<StopTestTactic>();
    auto stop_test_tactic_2 = std::make_shared<StopTestTactic>();
    auto stop_test_tactic_3 = std::make_shared<StopTestTactic>();

    do
    {
        yield({stop_test_tactic_1, stop_test_tactic_2, stop_test_tactic_3});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, HaltTestPlay> factory;
