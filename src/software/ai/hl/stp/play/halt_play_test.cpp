#include "software/ai/hl/stp/play/halt_play.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/tactic/stop_tactic.h"
#include "software/test_util/test_util.h"


TEST(StopPlayTest, test_example_play_invariant_always_holds)
{
    World world = ::TestUtil::createBlankTestingWorld();

    HaltPlay halt_play;
    EXPECT_TRUE(halt_play.invariantHolds(world));
}

TEST(StopPlayTest, test_stop_play_returns_correct_tactics)
{
    World world = ::TestUtil::createBlankTestingWorld();

    HaltPlay halt_play;
    auto tactics = halt_play.getTactics(world);

    // Make sure something was returned
    EXPECT_TRUE(tactics);

    // Make sure the expected number of tactics was returned
    EXPECT_EQ((*tactics).size(), 6);

    // Make sure each tactic is an ExampleTactic
    for (const auto &t : *tactics)
    {
        try
        {
            StopTactic *unused;
            unused = dynamic_cast<StopTactic *>(t.get());
        }
        catch (...)
        {
            ADD_FAILURE() << "StopTactic was not returned by the StopPlay!";
        }
    }
}
