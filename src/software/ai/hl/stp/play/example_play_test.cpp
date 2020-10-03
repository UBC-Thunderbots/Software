#include "software/ai/hl/stp/play/example_play.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/tactic/move_tactic.h"
#include "software/test_util/test_util.h"

TEST(ExamplePlayTest, test_example_play_never_applicable)
{
    World world = ::TestUtil::createBlankTestingWorld();

    ExamplePlay example_play;
    EXPECT_FALSE(example_play.isApplicable(world));
}

TEST(ExamplePlayTest, test_example_play_invariant_always_holds)
{
    World world = ::TestUtil::createBlankTestingWorld();

    ExamplePlay example_play;
    EXPECT_TRUE(example_play.invariantHolds(world));
}

TEST(ExamplePlayTest, test_example_play_returns_correct_tactics)
{
    World world = ::TestUtil::createBlankTestingWorld();

    ExamplePlay example_play;
    auto tactics = example_play.getTactics(world);

    // Make sure the expected number of tactics was returned
    EXPECT_EQ((tactics).size(), 6);

    // Make sure each tactic is an ExampleTactic
    for (const auto& t : tactics)
    {
        // The result of the dynamic_cast will be a nullptr if the pointer does not
        // actually point to a conrete type of MoveTactic
        EXPECT_NE(dynamic_cast<MoveTactic*>(t.get()), nullptr);
    }
}
