#include <gtest/gtest.h>

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "test/test_util/test_util.h"

TEST(MovePlayTest, test_move_play_always_applicable)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    auto move_play = PlayFactory::createPlay("Move Play");
    EXPECT_TRUE(move_play->isApplicable(world));
}

TEST(MovePlayTest, test_move_play_invariant_always_holds)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    auto move_play = PlayFactory::createPlay("Move Play");
    EXPECT_TRUE(move_play->invariantHolds(world));
}

TEST(MovePlayTest, test_move_play_returns_correct_tactics)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    auto move_play = PlayFactory::createPlay("Move Play");
    auto tactics   = move_play->getTactics(world);

    // Make sure something was returned
    EXPECT_TRUE(tactics);

    // Make sure the expected number of tactics was returned
    EXPECT_EQ((*tactics).size(), 6);

    // Make sure each tactic is a MoveTactic
    for (const auto& t : *tactics)
    {
        // The result of the dynamic_cast will be a nullptr if the pointer does not
        // actually point to a conrete type of MoveTactic
        EXPECT_NE(dynamic_cast<MoveTactic*>(t.get()), nullptr);
    }
}
