#include "software/proto/message_translation/ssl_referee.h"

#include <gtest/gtest.h>

#include "software/proto/ssl_referee.pb.h"
#include "software/world/team_colour.h"

class RefereeStageTest
    : public ::testing::TestWithParam<std::tuple<RefereeStage, SSL_Referee_Stage>>
{
};

TEST_P(RefereeStageTest, test_refbox_stage)
{
    RefereeStage expected    = std::get<0>(GetParam());
    SSL_Referee_Stage input = std::get<1>(GetParam());

    SSL_Referee ref;
    ref.set_stage(input);

    ASSERT_EQ(expected, createRefereeStage(ref));
}

INSTANTIATE_TEST_CASE_P(
    RefereeStageTests, RefereeStageTest,
    ::testing::Values(
        std::make_tuple(RefereeStage::NORMAL_FIRST_HALF_PRE,
                        SSL_Referee_Stage_NORMAL_FIRST_HALF_PRE),
        std::make_tuple(RefereeStage::NORMAL_FIRST_HALF,
                        SSL_Referee_Stage_NORMAL_FIRST_HALF),
        std::make_tuple(RefereeStage::NORMAL_HALF_TIME,
                        SSL_Referee_Stage_NORMAL_HALF_TIME),
        std::make_tuple(RefereeStage::NORMAL_SECOND_HALF_PRE,
                        SSL_Referee_Stage_NORMAL_SECOND_HALF_PRE),
        std::make_tuple(RefereeStage::NORMAL_SECOND_HALF,
                        SSL_Referee_Stage_NORMAL_SECOND_HALF),
        std::make_tuple(RefereeStage::EXTRA_TIME_BREAK,
                        SSL_Referee_Stage_EXTRA_TIME_BREAK),
        std::make_tuple(RefereeStage::EXTRA_FIRST_HALF_PRE,
                        SSL_Referee_Stage_EXTRA_FIRST_HALF_PRE),
        std::make_tuple(RefereeStage::EXTRA_FIRST_HALF,
                        SSL_Referee_Stage_EXTRA_FIRST_HALF),
        std::make_tuple(RefereeStage::EXTRA_HALF_TIME, SSL_Referee_Stage_EXTRA_HALF_TIME),
        std::make_tuple(RefereeStage::EXTRA_SECOND_HALF_PRE,
                        SSL_Referee_Stage_EXTRA_SECOND_HALF_PRE),
        std::make_tuple(RefereeStage::EXTRA_SECOND_HALF,
                        SSL_Referee_Stage_EXTRA_SECOND_HALF),
        std::make_tuple(RefereeStage::PENALTY_SHOOTOUT_BREAK,
                        SSL_Referee_Stage_PENALTY_SHOOTOUT_BREAK),
        std::make_tuple(RefereeStage::PENALTY_SHOOTOUT,
                        SSL_Referee_Stage_PENALTY_SHOOTOUT),
        std::make_tuple(RefereeStage::POST_GAME, SSL_Referee_Stage_POST_GAME)));

class RefereeCommandTest
    : public ::testing::TestWithParam<
          std::tuple<RefereeCommand, SSL_Referee_Command, TeamColour>>
{
};

TEST_P(RefereeCommandTest, test_referee_command)
{
    RefereeCommand expected  = std::get<0>(GetParam());
    SSL_Referee_Command input = std::get<1>(GetParam());
    TeamColour team_colour    = std::get<2>(GetParam());

    SSL_Referee ref;
    ref.set_command(input);

    ASSERT_EQ(expected, createRefereeCommand(ref, team_colour));
}

INSTANTIATE_TEST_CASE_P(
    RefereeCommandTests, RefereeCommandTest,
    ::testing::Values(
        // common to yellow and blue
        std::make_tuple(RefereeCommand::HALT, SSL_Referee_Command_HALT,
                        TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::STOP, SSL_Referee_Command_STOP,
                        TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::NORMAL_START, SSL_Referee_Command_NORMAL_START,
                        TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::FORCE_START, SSL_Referee_Command_FORCE_START,
                        TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::HALT, SSL_Referee_Command_HALT,
                        TeamColour::BLUE),
        std::make_tuple(RefereeCommand::STOP, SSL_Referee_Command_STOP,
                        TeamColour::BLUE),
        std::make_tuple(RefereeCommand::NORMAL_START, SSL_Referee_Command_NORMAL_START,
                        TeamColour::BLUE),
        std::make_tuple(RefereeCommand::FORCE_START, SSL_Referee_Command_FORCE_START,
                        TeamColour::BLUE),

        // kickoff
        std::make_tuple(RefereeCommand::PREPARE_KICKOFF_US,
                        SSL_Referee_Command_PREPARE_KICKOFF_YELLOW, TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::PREPARE_KICKOFF_US,
                        SSL_Referee_Command_PREPARE_KICKOFF_BLUE, TeamColour::BLUE),
        std::make_tuple(RefereeCommand::PREPARE_KICKOFF_THEM,
                        SSL_Referee_Command_PREPARE_KICKOFF_BLUE, TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::PREPARE_KICKOFF_THEM,
                        SSL_Referee_Command_PREPARE_KICKOFF_YELLOW, TeamColour::BLUE),

        // penalty
        std::make_tuple(RefereeCommand::PREPARE_PENALTY_US,
                        SSL_Referee_Command_PREPARE_PENALTY_YELLOW, TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::PREPARE_PENALTY_US,
                        SSL_Referee_Command_PREPARE_PENALTY_BLUE, TeamColour::BLUE),
        std::make_tuple(RefereeCommand::PREPARE_PENALTY_THEM,
                        SSL_Referee_Command_PREPARE_PENALTY_BLUE, TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::PREPARE_PENALTY_THEM,
                        SSL_Referee_Command_PREPARE_PENALTY_YELLOW, TeamColour::BLUE),

        // direct free
        std::make_tuple(RefereeCommand::DIRECT_FREE_US,
                        SSL_Referee_Command_DIRECT_FREE_YELLOW, TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::DIRECT_FREE_US,
                        SSL_Referee_Command_DIRECT_FREE_BLUE, TeamColour::BLUE),
        std::make_tuple(RefereeCommand::DIRECT_FREE_THEM,
                        SSL_Referee_Command_DIRECT_FREE_BLUE, TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::DIRECT_FREE_THEM,
                        SSL_Referee_Command_DIRECT_FREE_YELLOW, TeamColour::BLUE),

        // indirect free
        std::make_tuple(RefereeCommand::INDIRECT_FREE_US,
                        SSL_Referee_Command_INDIRECT_FREE_YELLOW, TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::INDIRECT_FREE_US,
                        SSL_Referee_Command_INDIRECT_FREE_BLUE, TeamColour::BLUE),
        std::make_tuple(RefereeCommand::INDIRECT_FREE_THEM,
                        SSL_Referee_Command_INDIRECT_FREE_BLUE, TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::INDIRECT_FREE_THEM,
                        SSL_Referee_Command_INDIRECT_FREE_YELLOW, TeamColour::BLUE),

        // timeout
        std::make_tuple(RefereeCommand::TIMEOUT_US, SSL_Referee_Command_TIMEOUT_YELLOW,
                        TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::TIMEOUT_US, SSL_Referee_Command_TIMEOUT_BLUE,
                        TeamColour::BLUE),
        std::make_tuple(RefereeCommand::TIMEOUT_THEM, SSL_Referee_Command_TIMEOUT_BLUE,
                        TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::TIMEOUT_THEM, SSL_Referee_Command_TIMEOUT_YELLOW,
                        TeamColour::BLUE),

        // goal
        std::make_tuple(RefereeCommand::GOAL_US, SSL_Referee_Command_GOAL_YELLOW,
                        TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::GOAL_US, SSL_Referee_Command_GOAL_BLUE,
                        TeamColour::BLUE),
        std::make_tuple(RefereeCommand::GOAL_THEM, SSL_Referee_Command_GOAL_BLUE,
                        TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::GOAL_THEM, SSL_Referee_Command_GOAL_YELLOW,
                        TeamColour::BLUE),

        // ball placement
        std::make_tuple(RefereeCommand::BALL_PLACEMENT_US,
                        SSL_Referee_Command_BALL_PLACEMENT_YELLOW, TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::BALL_PLACEMENT_US,
                        SSL_Referee_Command_BALL_PLACEMENT_BLUE, TeamColour::BLUE),
        std::make_tuple(RefereeCommand::BALL_PLACEMENT_THEM,
                        SSL_Referee_Command_BALL_PLACEMENT_BLUE, TeamColour::YELLOW),
        std::make_tuple(RefereeCommand::BALL_PLACEMENT_THEM,
                        SSL_Referee_Command_BALL_PLACEMENT_YELLOW, TeamColour::BLUE)));


TEST(BallPlacementPointTest, test_ball_placement_point)
{
    SSL_Referee ref;
    auto ref_point = std::make_unique<SSL_Referee_Point>();

    ref_point->set_x(100);
    ref_point->set_y(200);

    *(ref.mutable_designated_position()) = *ref_point;

    ASSERT_EQ(Point(100, 200), getBallPlacementPoint(ref));
}
