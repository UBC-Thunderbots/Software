#include "software/ai/hl/stp/stp.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <exception>

#include "software/ai/hl/stp/play/halt_play.h"
#include "software/test_util/test_util.h"

class STPTest : public ::testing::Test
{
   public:
    STPTest()
        : world(::TestUtil::createBlankTestingWorld()),
          // Give an explicit seed to STP so that our tests are deterministic
          stp(ai_config)
    {
    }

   protected:
    void SetUp() override
    {
        // Explicitly setting override AI Play to be false
        ai_config.mutable_ai_control_config()->clear_override_ai_play();
    }

    World world;
    TbotsProto::AiConfig ai_config;
    STP stp;
};

TEST_F(STPTest, test_get_play_info)
{
    world = ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    world = ::TestUtil::setFriendlyRobotPositions(world, {Point(0, 0), Point(1, 0)},
                                                  Timestamp::fromSeconds(0));
    world.updateRefereeCommand(RefereeCommand::HALT);
    stp.getIntents(world);

    auto play_info_msg = stp.getPlayInfo();

    std::string expected_play_name, expected_tactic_name;
    expected_play_name   = "HaltPlay";
    expected_tactic_name = "StopTactic";

    TbotsProto::PlayInfo expected_play_info_msg = TbotsProto::PlayInfo();
    expected_play_info_msg.mutable_play()->set_play_name(expected_play_name);
    TbotsProto::PlayInfo_Tactic expected_tactic = TbotsProto::PlayInfo_Tactic();
    expected_tactic.set_tactic_name(expected_tactic_name);
    (*expected_play_info_msg.mutable_robot_tactic_assignment())[0] = expected_tactic;
    (*expected_play_info_msg.mutable_robot_tactic_assignment())[1] = expected_tactic;

    EXPECT_EQ(play_info_msg.play().play_name(), expected_play_name);
    EXPECT_EQ(play_info_msg.robot_tactic_assignment_size(), 2);
    EXPECT_EQ((*play_info_msg.mutable_robot_tactic_assignment())[0].tactic_name(),
              expected_tactic.tactic_name());
    EXPECT_EQ((*play_info_msg.mutable_robot_tactic_assignment())[1].tactic_name(),
              expected_tactic.tactic_name());
}
