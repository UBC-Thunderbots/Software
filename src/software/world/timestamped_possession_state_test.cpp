#include "software/world/timestamped_possession_state.h"

#include <gtest/gtest.h>

TEST(TimestampedPossessionStateTest, test_sequence_of_timestamps)
{
    TimestampedPossessionState timestamped_possession_state;
    timestamped_possession_state.updateState({}, Timestamp::fromSeconds(1.0));
    EXPECT_EQ(Timestamp::fromSeconds(1.0), timestamped_possession_state.timestamp());
    timestamped_possession_state.updateState({}, Timestamp::fromSeconds(1.0));
    EXPECT_EQ(Timestamp::fromSeconds(1.0), timestamped_possession_state.timestamp());
    timestamped_possession_state.updateState({}, Timestamp::fromSeconds(3.2));
    EXPECT_EQ(Timestamp::fromSeconds(3.2), timestamped_possession_state.timestamp());
}

TEST(TimestampedPossessionStateTest, test_older_of_timestamps)
{
    TimestampedPossessionState timestamped_possession_state;
    timestamped_possession_state.updateState({}, Timestamp::fromSeconds(2.0));
    EXPECT_EQ(Timestamp::fromSeconds(2.0), timestamped_possession_state.timestamp());
    EXPECT_THROW(
        timestamped_possession_state.updateState({}, Timestamp::fromSeconds(1.0)),
        std::invalid_argument);
}

TEST(TimestampedPossessionStateTest, test_get_robots_with_possession_one_robot)
{
    TimestampedPossessionState timestamped_possession_state;
    timestamped_possession_state.updateState(
        {RobotIdWithTeamSide{.id = 0, .team_side = TeamSide::FRIENDLY}},
        Timestamp::fromSeconds(1.0));
    EXPECT_EQ(std::vector<RobotIdWithTeamSide>(
                  {RobotIdWithTeamSide{.id = 0, .team_side = TeamSide::FRIENDLY}}),
              timestamped_possession_state.getRobotsWithPossession());
    timestamped_possession_state.updateState(
        {RobotIdWithTeamSide{.id = 1, .team_side = TeamSide::ENEMY}},
        Timestamp::fromSeconds(1.2));
    EXPECT_EQ(std::vector<RobotIdWithTeamSide>(
                  {RobotIdWithTeamSide{.id = 1, .team_side = TeamSide::ENEMY}}),
              timestamped_possession_state.getRobotsWithPossession());
}

TEST(TimestampedPossessionStateTest, test_get_robots_with_possession_multiple_robots)
{
    TimestampedPossessionState timestamped_possession_state;
    timestamped_possession_state.updateState(
        {RobotIdWithTeamSide{.id = 0, .team_side = TeamSide::FRIENDLY}},
        Timestamp::fromSeconds(2.1));
    EXPECT_EQ(std::vector<RobotIdWithTeamSide>(
                  {RobotIdWithTeamSide{.id = 0, .team_side = TeamSide::FRIENDLY}}),
              timestamped_possession_state.getRobotsWithPossession());
    timestamped_possession_state.updateState(
        {RobotIdWithTeamSide{.id = 1, .team_side = TeamSide::ENEMY},
         RobotIdWithTeamSide{.id = 2, .team_side = TeamSide::FRIENDLY}},
        Timestamp::fromSeconds(2.2));
    EXPECT_EQ(std::vector<RobotIdWithTeamSide>(
                  {RobotIdWithTeamSide{.id = 1, .team_side = TeamSide::ENEMY},
                   RobotIdWithTeamSide{.id = 2, .team_side = TeamSide::FRIENDLY}}),
              timestamped_possession_state.getRobotsWithPossession());
}

TEST(TimestampedPossessionStateTest, test_get_robots_with_possession_no_robots)
{
    TimestampedPossessionState timestamped_possession_state;
    timestamped_possession_state.updateState(
        {RobotIdWithTeamSide{.id = 0, .team_side = TeamSide::FRIENDLY}},
        Timestamp::fromSeconds(1.0));
    EXPECT_EQ(std::vector<RobotIdWithTeamSide>(
                  {RobotIdWithTeamSide{.id = 0, .team_side = TeamSide::FRIENDLY}}),
              timestamped_possession_state.getRobotsWithPossession());
    timestamped_possession_state.updateState({}, Timestamp::fromSeconds(1.2));
    EXPECT_EQ(std::vector<RobotIdWithTeamSide>({}),
              timestamped_possession_state.getRobotsWithPossession());
}

TEST(TimestampedPossessionStateTest, test_get_team_with_possession)
{
    TimestampedPossessionState timestamped_possession_state;
    timestamped_possession_state.updateState(
        {RobotIdWithTeamSide{.id = 0, .team_side = TeamSide::FRIENDLY}},
        Timestamp::fromSeconds(1.0));
    EXPECT_EQ(TeamSide::FRIENDLY, timestamped_possession_state.getTeamWithPossession());
    timestamped_possession_state.updateState({}, Timestamp::fromSeconds(1.2));
    EXPECT_EQ(std::nullopt, timestamped_possession_state.getTeamWithPossession());
    timestamped_possession_state.updateState(
        {RobotIdWithTeamSide{.id = 1, .team_side = TeamSide::ENEMY},
         RobotIdWithTeamSide{.id = 2, .team_side = TeamSide::FRIENDLY}},
        Timestamp::fromSeconds(3.2));
    EXPECT_EQ(std::nullopt, timestamped_possession_state.getTeamWithPossession());
}
