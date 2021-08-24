#include "software/ai/hl/stp/stp.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <exception>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/test_plays/halt_test_play.h"
#include "software/ai/hl/stp/play/test_plays/move_test_play.h"
#include "software/test_util/test_util.h"
#include "software/util/design_patterns/generic_factory.h"

class STPTest : public ::testing::Test
{
   public:
    STPTest()
        : world(::TestUtil::createBlankTestingWorld()),
          mutable_ai_control_config(std::make_shared<AiControlConfig>()),
          ai_control_config(
              std::const_pointer_cast<const AiControlConfig>(mutable_ai_control_config)),
          play_config(std::make_shared<const ThunderbotsConfig>()->getPlayConfig()),
          default_play_constructor([this]() -> std::unique_ptr<Play> {
              return std::make_unique<HaltTestPlay>(play_config);
          }),
          // Give an explicit seed to STP so that our tests are deterministic
          stp(default_play_constructor, ai_control_config, play_config, 123)
    {
    }

   protected:
    void SetUp() override
    {
        // Explicitly setting override AI Play to be false
        mutable_ai_control_config->getMutableOverrideAiPlay()->setValue(false);
    }

    World world;
    std::shared_ptr<AiControlConfig> mutable_ai_control_config;
    std::shared_ptr<const AiControlConfig> ai_control_config;
    std::shared_ptr<const PlayConfig> play_config;
    std::function<std::unique_ptr<Play>()> default_play_constructor;
    STP stp;
};

TEST_F(STPTest, test_only_test_plays_are_registered_in_play_factory)
{
    auto play_names = GenericFactory<std::string, Play, PlayConfig>::getRegisteredNames();
    EXPECT_EQ(2, play_names.size());
    EXPECT_EQ(std::count(play_names.begin(), play_names.end(), TYPENAME(MoveTestPlay)),
              1);
    EXPECT_EQ(std::count(play_names.begin(), play_names.end(), TYPENAME(HaltTestPlay)),
              1);
}

TEST_F(STPTest, test_exception_thrown_when_no_play_applicable)
{
    // Put the ball where both its x and y coordinates are negative. Neither test Play
    // is applicable in this case
    world = world =
        ::TestUtil::setBallPosition(world, Point(-1, -1), Timestamp::fromSeconds(0));
    EXPECT_THROW(stp.calculateNewPlay(world), std::runtime_error);
}

TEST_F(STPTest, test_calculate_new_play_when_one_play_valid)
{
    // Only the HaltTestPlay should be applicable
    world = ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    auto play = stp.calculateNewPlay(world);
    EXPECT_TRUE(play);
    EXPECT_EQ(objectTypeName(*play), TYPENAME(HaltTestPlay));
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
        actual_play_names.emplace_back(objectTypeName(*play));
    }

    std::vector<std::string> expected_play_names = {
        TYPENAME(MoveTestPlay), TYPENAME(MoveTestPlay), TYPENAME(HaltTestPlay),
        TYPENAME(HaltTestPlay), TYPENAME(HaltTestPlay), TYPENAME(MoveTestPlay),
        TYPENAME(MoveTestPlay), TYPENAME(MoveTestPlay), TYPENAME(MoveTestPlay),
        TYPENAME(HaltTestPlay),
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
    EXPECT_EQ(*(stp.getCurrentPlayName()), TYPENAME(HaltTestPlay));
}

TEST_F(
    STPTest,
    test_play_assignment_from_one_play_to_another_when_current_play_invariant_no_longer_holds)
{
    // Only the HaltTestPlay should be applicable
    world = ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), TYPENAME(HaltTestPlay));

    // The HaltTestPlays invariant should no longer hold, and the MoveTestPlay should now
    // be applicable
    world = ::TestUtil::setBallPosition(
        world, world.field().enemyCornerNeg() + Vector(1, 0), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), TYPENAME(MoveTestPlay));
}

TEST_F(STPTest, test_play_assignment_from_one_play_to_another_when_current_play_is_done)
{
    // Only the MoveTestPlay should be applicable
    world = ::TestUtil::setBallPosition(world, Point(1, -1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), TYPENAME(MoveTestPlay));

    // Now only the HaltTestPlay should be applicable, and the MoveTestPlay's invariant
    // no longer holds, so we expect the current play to become the HaltTestPlay
    world = ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), TYPENAME(HaltTestPlay));
}

TEST_F(STPTest, test_fallback_play_assigned_when_no_new_plays_are_applicable)
{
    // TODO: Update this test when https://github.com/UBC-Thunderbots/Software/issues/396
    // is completed. For now, we just check that the previous play remains assigned

    // Only the HaltTestPlay should be applicable
    world = ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), TYPENAME(HaltTestPlay));

    // Put the ball where both its x and y coordinates are negative. Neither test Play
    // is applicable in this case
    world = ::TestUtil::setBallPosition(world, Point(-1, -1), Timestamp::fromSeconds(0));
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), TYPENAME(HaltTestPlay));
}

TEST_F(STPTest, test_get_play_info)
{
    // Only the HaltTestPlay should be applicable
    world = ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    world = ::TestUtil::setFriendlyRobotPositions(world, {Point(0, 0), Point(1, 0)},
                                                  Timestamp::fromSeconds(0));
    world.updateRefereeCommand(RefereeCommand::HALT);
    stp.getIntents(world);
    EXPECT_EQ(*(stp.getCurrentPlayName()), TYPENAME(HaltTestPlay));

    auto play_info_msg = stp.getPlayInfoProto();
    std::string expected_referee_command, expected_play_name, expected_tactic_name;
    expected_referee_command                                  = "HALT";
    expected_play_name                                        = "HaltTestPlay";
    expected_tactic_name                                      = "StopTestTactic";
//    std::vector<std::string> expected_robot_tactic_assignment = {
//        "Robot 0  -  StopTestTactic", "Robot 1  -  StopTestTactic"};
    PlayInfoProto expected_play_info_msg = PlayInfoProto();
    expected_play_info_msg.mutable_game_state()->set_referee_command_name(expected_referee_command);
    expected_play_info_msg.mutable_play()->set_play_name(expected_play_name);
    PlayInfoProto_Tactic expected_tactic = PlayInfoProto_Tactic();
    expected_tactic.set_tactic_name(expected_tactic_name);

    (*expected_play_info_msg.mutable_robot_tactic_assignment())[0] = expected_tactic;
    (*expected_play_info_msg.mutable_robot_tactic_assignment())[1] = expected_tactic;
    EXPECT_EQ(play_info_msg.game_state().referee_command_name(), expected_referee_command);
    EXPECT_EQ(play_info_msg.play().play_name(), expected_play_name);
    EXPECT_EQ(play_info_msg.robot_tactic_assignment_size(), 2);
    EXPECT_EQ(play_info_msg.robot_tactic_assignment().find(0), expected_tactic.tactic_name());
}
