#include <gtest/gtest.h>

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "test/test_util/test_util.h"

TEST(ExamplePlayTest, test_example_play_always_applicable)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    auto example_play = PlayFactory::createPlay("Example Play");
    EXPECT_TRUE(example_play->isApplicable(world));
}

TEST(ExamplePlayTest, test_example_play_invariant_always_holds)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    auto example_play = PlayFactory::createPlay("Example Play");
    EXPECT_TRUE(example_play->invariantHolds(world));
}

TEST(ExamplePlayTest, test_example_play_returns_correct_tactics)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    auto example_play = PlayFactory::createPlay("Example Play");
    auto tactics      = example_play->getTactics(world);

    // Make sure something was returned
    EXPECT_TRUE(tactics);

    // Make sure the expected number of tactics was returned
    EXPECT_EQ((*tactics).size(), 6);

    // Make sure each tactic is an ExampleTactic
    for (const auto& t : *tactics)
    {
        // The result of the dynamic_cast will be a nullptr if the pointer does not
        // actually point to a conrete type of MoveTactic
        EXPECT_NE(dynamic_cast<MoveTactic*>(t.get()), nullptr);
    }
}
