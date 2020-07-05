#include "software/ai/hl/stp/stp.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <exception>

#include "software/ai/hl/stp/play/test_plays/halt_test_play.h"
#include "software/ai/hl/stp/play/test_plays/move_test_play.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/test_util/test_util.h"
#include "software/util/design_patterns/generic_factory.h"

class STPTest : public ::testing::Test
{
   public:
    STPTest() : stp([]() { return nullptr; }, DynamicParameters->getAIControlConfig(), 0)
    {
    }

   protected:
    void SetUp() override
    {
        auto default_play_constructor = []() -> std::unique_ptr<Play> {
            return std::make_unique<HaltTestPlay>();
        };
        // Give an explicit seed to STP so that our tests are deterministic
        stp   = STP(default_play_constructor, DynamicParameters->getAIControlConfig(), 0);
        world = ::TestUtil::createBlankTestingWorld();
    }

    STP stp;
    World world = ::TestUtil::createBlankTestingWorld();
};

TEST_F(STPTest, test_only_test_plays_are_registered_in_play_factory)
{
    auto play_names = GenericFactory<std::string, Play>::getRegisteredNames();
    EXPECT_EQ(2, play_names.size());
    EXPECT_EQ(std::count(play_names.begin(), play_names.end(), MoveTestPlay::name), 1);
    EXPECT_EQ(std::count(play_names.begin(), play_names.end(), HaltTestPlay::name), 1);
}

TEST_F(STPTest, test_exception_thrown_when_no_play_applicable)
{
    // Put the ball where both its x and y coordinates are negative. Neither test Play
    // is applicable in this case
    world = ::TestUtil::setBallPosition(world, Point(-1, -1), Timestamp::fromSeconds(0));
    EXPECT_THROW(stp.calculateNewPlay(world), std::runtime_error);
}

TEST_F(STPTest, test_calculate_new_play_when_one_play_valid)
{
    // Only the HaltTestPlay should be applicable
    world = ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    auto play = stp.calculateNewPlay(world);
    EXPECT_TRUE(play);
    EXPECT_EQ(play->getName(), HaltTestPlay::name);
}

TEST_F(STPTest, test_calculate_new_play_when_multiple_plays_valid)
{
    // Both HaltTestPlay and MoveTestPlay should be applicable
    world = ::TestUtil::setBallPosition(world, Point(1, 1), Timestamp::fromSeconds(0));

    // We expect a random selection of the plays that are applicable. This test should be
    // deterministic because we have provided a seed for this test (in the setUp function)

    std::unique_ptr<Play> play;
    std::vector<std::string> actual_play_names;

    for (unsigned int i = 0; i < 10; i++)
    {
        play = stp.calculateNewPlay(world);
        actual_play_names.emplace_back(play->getName());
    }

    std::vector<std::string> expected_play_names = {
        MoveTestPlay::name, MoveTestPlay::name, MoveTestPlay::name, MoveTestPlay::name,
        MoveTestPlay::name, MoveTestPlay::name, MoveTestPlay::name, MoveTestPlay::name,
        HaltTestPlay::name, MoveTestPlay::name,
    };

    EXPECT_EQ(expected_play_names, actual_play_names);
}

TEST_F(STPTest, test_current_play_initially_unassigned)
{
    EXPECT_EQ(stp.getCurrentPlayName(), std::nullopt);
}

// Test that we can successfully assign Plays when there is currently no Play assigned
TEST_F(STPTest, test_play_assignment_transition_from_unassigned_to_assigned)
{
    // Only the HaltTestPlay should be applicable
    world = ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), HaltTestPlay::name);
}

TEST_F(
    STPTest,
    test_play_assignment_from_one_play_to_another_when_current_play_invariant_no_longer_holds)
{
    // Only the HaltTestPlay should be applicable
    world = ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), HaltTestPlay::name);

    // The HaltTestPlays invariant should no longer hold, and the MoveTestPlay should now
    // be applicable
    world = ::TestUtil::setBallPosition(
        world, world.field().enemyCornerNeg() + Vector(1, 0), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), MoveTestPlay::name);
}

TEST_F(STPTest, test_play_assignment_from_one_play_to_another_when_current_play_is_done)
{
    // Only the MoveTestPlay should be applicable
    world = ::TestUtil::setBallPosition(world, Point(1, -1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), MoveTestPlay::name);

    // Now only the HaltTestPlay should be applicable, and the MoveTestPlay's invariant
    // no longer holds, so we expect the current play to become the HaltTestPlay
    world = ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), HaltTestPlay::name);
}

TEST_F(STPTest, test_fallback_play_assigned_when_no_new_plays_are_applicable)
{
    // TODO: Update this test when https://github.com/UBC-Thunderbots/Software/issues/396
    // is completed. For now, we just check that the previous play remains assigned

    // Only the HaltTestPlay should be applicable
    world = ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), HaltTestPlay::name);

    // Put the ball where both its x and y coordinates are negative. Neither test Play
    // is applicable in this case
    world = ::TestUtil::setBallPosition(world, Point(-1, -1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), HaltTestPlay::name);
}

TEST_F(STPTest, test_get_play_info)
{
    // Only the HaltTestPlay should be applicable
    world = ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    world = ::TestUtil::setFriendlyRobotPositions(world, {Point(0, 0), Point(1, 0)},
                                                  Timestamp::fromSeconds(0));
    world.mutableGameState().game_state = RefboxGameState::HALT;
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), HaltTestPlay::name);

    auto play_info = stp.getPlayInfo();
    PlayInfo expected_play_info;
    std::string expected_refbox_game_state, expected_play_name;
    expected_refbox_game_state                                       = "HALT";
    expected_play_name                                               = "Halt Test Play";
    std::unordered_set<std::string> expected_robot_tactic_assignment = {
        "Robot 0  -  Stop Test Tactic", "Robot 1  -  Stop Test Tactic"};
    expected_play_info = PlayInfo(expected_refbox_game_state, expected_play_name,
                                  expected_robot_tactic_assignment);
    EXPECT_EQ(play_info, expected_play_info);
}
