#include "ai/hl/stp/play/stop_play.h"

#include <gtest/gtest.h>

#include "ai/hl/stp/tactic/stop_tactic.h"
#include "test/test_util/test_util.h"

TEST(StopPlayTest, test_example_play_invariant_always_holds)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    StopPlay example_play;
    EXPECT_TRUE(example_play.invariantHolds(world));
}

TEST(StopPlayTest, test_stop_play_returns_correct_tactics)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    StopPlay example_play;
    auto tactics = example_play.getTactics(world);

    // Make sure something was returned
    EXPECT_TRUE(tactics);

    // Make sure the expected number of tactics was returned
    EXPECT_EQ((*tactics).size(), 6);

    // Make sure each tactic is an ExampleTactic
    for (const auto& t : *tactics)
    {
        try
        {
            dynamic_cast<StopTactic*>(t.get());
        }
        catch (...)
        {
            ADD_FAILURE() << "StopTactic was not returned by the StopPlay!";
        }
    }
}
