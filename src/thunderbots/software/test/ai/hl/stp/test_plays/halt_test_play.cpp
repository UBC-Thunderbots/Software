#include "test/ai/hl/stp/test_plays/halt_test_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "geom/util.h"
#include "test/ai/hl/stp/test_tactics/stop_test_tactic.h"

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
    return contains(
        Rectangle(world.field().enemyCornerNeg(), world.field().friendlyCornerPos()),
        world.ball().position());
}

void HaltTestPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    auto stop_test_tactic_1 = std::make_shared<StopTestTactic>();
    auto stop_test_tactic_2 = std::make_shared<StopTestTactic>();
    auto stop_test_tactic_3 = std::make_shared<StopTestTactic>();

    do
    {
        stop_test_tactic_1->updateParams();
        stop_test_tactic_2->updateParams();
        stop_test_tactic_3->updateParams();

        yield({stop_test_tactic_1, stop_test_tactic_2, stop_test_tactic_3});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<HaltTestPlay> factory;
