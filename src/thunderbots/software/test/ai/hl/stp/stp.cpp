#include "ai/hl/stp/stp.h"

#include <gtest/gtest.h>
#include <test/ai/hl/stp/test_tactics/move_test_tactic.h>

#include <algorithm>
#include <exception>

#include "ai/hl/stp/play/play_factory.h"
#include "test/ai/hl/stp/test_plays/move_test_play.h"
#include "test/ai/hl/stp/test_plays/stop_test_play.h"
#include "test/test_util/test_util.h"

class STPTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        // Give an explicit seed to STP so that our tests are deterministic
        stp   = STP(0);
        world = ::Test::TestUtil::createBlankTestingWorld();
    }

    STP stp;
    World world;
};

TEST_F(STPTest, test_only_test_plays_are_registered_in_play_factory)
{
    auto play_names = PlayFactory::getRegisteredPlayNames();
    EXPECT_EQ(play_names.size(), 2);
    EXPECT_EQ(std::count(play_names.begin(), play_names.end(), MoveTestPlay::name), 1);
    EXPECT_EQ(std::count(play_names.begin(), play_names.end(), StopTestPlay::name), 1);
}

TEST_F(STPTest, test_exception_thrown_when_no_play_applicable)
{
    // Put the ball where both its x and y coordinates are negative. Neither test Play
    // is applicable in this case
    world = ::Test::TestUtil::setBallPosition(world, Point(-1, -1),
                                              Timestamp::fromSeconds(0));
    EXPECT_THROW(stp.calculateNewPlay(world), std::runtime_error);
}

TEST_F(STPTest, test_calculate_new_play_when_one_play_valid)
{
    // Only the StopTestPlay should be applicable
    world =
        ::Test::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    auto play = stp.calculateNewPlay(world);
    EXPECT_TRUE(play);
    EXPECT_EQ(play->getName(), StopTestPlay::name);
}

TEST_F(STPTest, test_calculate_new_play_when_multiple_plays_valid)
{
    // Both StopTestPlay and MoveTestPlay should be applicable
    world =
        ::Test::TestUtil::setBallPosition(world, Point(1, 1), Timestamp::fromSeconds(0));

    // We expect a random selection of the plays that are applicable. This test should be
    // deterministic because we have provided a seed for this test (in the setUp function)

    std::unique_ptr<Play> play;
    for (int i = 0; i < 8; i++)
    {
        play = stp.calculateNewPlay(world);
        EXPECT_TRUE(play);
        EXPECT_EQ(play->getName(), MoveTestPlay::name);
    }

    play = stp.calculateNewPlay(world);
    EXPECT_TRUE(play);
    EXPECT_EQ(play->getName(), StopTestPlay::name);

    for (int i = 0; i < 2; i++)
    {
        play = stp.calculateNewPlay(world);
        EXPECT_TRUE(play);
        EXPECT_EQ(play->getName(), MoveTestPlay::name);
    }

    for (int i = 0; i < 3; i++)
    {
        play = stp.calculateNewPlay(world);
        EXPECT_TRUE(play);
        EXPECT_EQ(play->getName(), StopTestPlay::name);
    }

    play = stp.calculateNewPlay(world);
    EXPECT_TRUE(play);
    EXPECT_EQ(play->getName(), MoveTestPlay::name);

    play = stp.calculateNewPlay(world);
    EXPECT_TRUE(play);
    EXPECT_EQ(play->getName(), StopTestPlay::name);

    play = stp.calculateNewPlay(world);
    EXPECT_TRUE(play);
    EXPECT_EQ(play->getName(), MoveTestPlay::name);
}

TEST_F(STPTest, test_current_play_initially_unassigned)
{
    EXPECT_EQ(stp.getCurrentPlayName(), std::nullopt);
}

// Test that we can successfully assign Plays when there is currently no Play assigned
TEST_F(STPTest, test_play_assignment_transition_from_unassigned_to_assigned)
{
    // Only the StopTestPlay should be applicable
    world =
        ::Test::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), StopTestPlay::name);
}

TEST_F(
    STPTest,
    test_play_assignment_from_one_play_to_another_when_current_play_invariant_no_longer_holds)
{
    // Only the StopTestPlay should be applicable
    world =
        ::Test::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), StopTestPlay::name);

    // The StopTestPlays invariant should no longer hold, and the MoveTestPlay should now
    // be applicable
    world = ::Test::TestUtil::setBallPosition(
        world, world.field().enemyCornerNeg() + Vector(1, 0), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), MoveTestPlay::name);
}

TEST_F(STPTest, test_play_assignment_from_one_play_to_another_when_current_play_is_done)
{
    // Only the MoveTestPlay should be applicable
    world =
        ::Test::TestUtil::setBallPosition(world, Point(1, -1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), MoveTestPlay::name);

    // Now only the StopTestPlay should be applicable, and the MoveTestPlay's invariant
    // no longer holds, so we expect the current play to become the StopTestPlay
    world =
        ::Test::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), StopTestPlay::name);
}

TEST_F(STPTest, test_fallback_play_assigned_when_no_new_plays_are_applicable)
{
    // TODO: Update this test when https://github.com/UBC-Thunderbots/Software/issues/396
    // is completed. For now, we just check that the previous play remains assigned

    // Only the StopTestPlay should be applicable
    world =
        ::Test::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), StopTestPlay::name);

    // Put the ball where both its x and y coordinates are negative. Neither test Play
    // is applicable in this case
    world = ::Test::TestUtil::setBallPosition(world, Point(-1, -1),
                                              Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), StopTestPlay::name);
}
