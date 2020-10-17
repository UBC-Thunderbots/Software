#include "software/world/team.h"

#include <gtest/gtest.h>

#include <stdexcept>

class TeamTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        current_time = Timestamp::fromSeconds(123);

        one_second_future        = current_time + Duration::fromSeconds(1);
        two_seconds_future       = current_time + Duration::fromSeconds(2);
        two_seconds_100ms_future = current_time + Duration::fromMilliseconds(2100);
        three_seconds_future     = current_time + Duration::fromSeconds(3);

        one_second_past = current_time - Duration::fromSeconds(1);
    }

    Timestamp current_time;

    Timestamp one_second_future;
    Timestamp two_seconds_future;
    Timestamp two_seconds_100ms_future;
    Timestamp three_seconds_future;

    Timestamp one_second_past;
};

TEST_F(TeamTest, construction_with_expiry_duration)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    EXPECT_EQ(0, team.numRobots());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(std::vector<Robot>(), team.getAllRobots());
    EXPECT_EQ(Duration::fromMilliseconds(1000), team.getRobotExpiryBufferDuration());
}

TEST_F(TeamTest, construction_with_expiry_duration_and_team_robots)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1, robot_2};

    Team team = Team(robot_list, Duration::fromMilliseconds(1000));

    EXPECT_EQ(Duration::fromMilliseconds(1000), team.getRobotExpiryBufferDuration());

    EXPECT_EQ(3, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());
}

TEST_F(TeamTest, update_with_3_robots)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1, robot_2};

    team.updateRobots(robot_list);

    EXPECT_EQ(3, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());
}

TEST_F(TeamTest, update_with_new_team)
{
    Team team_update = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1, robot_2};

    team_update.updateRobots(robot_list);

    Team team = Team(Duration::fromMilliseconds(1000));

    team.updateState(team_update);

    EXPECT_EQ(3, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());

    EXPECT_EQ(team_update, team);
}

TEST_F(TeamTest, remove_expired_robots_at_current_time_so_no_robots_expire)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    team.updateRobots({robot_0, robot_1});

    team.removeExpiredRobots(current_time);

    EXPECT_EQ(2, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
}

TEST_F(TeamTest, remove_expired_robots_in_future_before_expiry_time_so_no_robots_expire)
{
    Team team = Team(Duration::fromMilliseconds(2100));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    team.updateRobots({robot_0, robot_1});

    team.removeExpiredRobots(two_seconds_future);

    EXPECT_EQ(2, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
}

TEST_F(TeamTest, remove_expired_robots_in_past_so_no_robots_expire)
{
    Team team = Team(Duration::fromMilliseconds(2000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    team.updateRobots({robot_0, robot_1});

    EXPECT_EQ(2, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
}

TEST_F(TeamTest, remove_expired_robots_in_future_so_all_robots_expire)
{
    Team team = Team(Duration::fromMilliseconds(2000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    team.updateRobots({robot_0, robot_1});

    team.removeExpiredRobots(two_seconds_100ms_future);

    EXPECT_EQ(0, team.numRobots());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
}

TEST_F(TeamTest, remove_expired_robots_in_future_so_1_robot_expires_1_robot_does_not)
{
    Team team = Team(Duration::fromMilliseconds(2000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), one_second_future);

    team.updateRobots({robot_0, robot_1});

    team.removeExpiredRobots(two_seconds_100ms_future);

    EXPECT_EQ(1, team.numRobots());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
}

TEST_F(TeamTest, removeRobotWithId_robot_with_id_on_team)
{
    Team team     = Team(Duration::fromMilliseconds(2000));
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), one_second_future);
    team.updateRobots({robot_0, robot_1});

    team.removeRobotWithId(0);

    EXPECT_EQ(1, team.getAllRobots().size());
    EXPECT_EQ(std::vector<Robot>{robot_1}, team.getAllRobots());
}

TEST_F(TeamTest, removeRobotWithId_robot_with_id_not_on_team)
{
    Team team     = Team(Duration::fromMilliseconds(2000));
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), one_second_future);
    team.updateRobots({robot_0, robot_1});

    team.removeRobotWithId(2);

    EXPECT_EQ(2, team.getAllRobots().size());
    EXPECT_EQ(std::vector<Robot>({robot_0, robot_1}), team.getAllRobots());
}


TEST_F(TeamTest, clear_all_robots)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1};

    team.updateRobots(robot_list);

    EXPECT_EQ(2, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());

    team.clearAllRobots();

    EXPECT_EQ(0, team.numRobots());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(std::vector<Robot>(), team.getAllRobots());
}

TEST_F(TeamTest, assign_goalie_starting_with_no_goalie)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1};

    team.updateRobots(robot_list);

    EXPECT_EQ(std::nullopt, team.goalie());

    team.assignGoalie(0);

    EXPECT_EQ(robot_0, team.goalie());
}

TEST_F(TeamTest, assign_goalie_starting_with_different_robot_as_goalie)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1};

    team.updateRobots(robot_list);
    team.assignGoalie(0);

    EXPECT_EQ(robot_0, team.goalie());

    team.assignGoalie(1);

    EXPECT_EQ(robot_1, team.goalie());
}

TEST_F(TeamTest, clear_goalie)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1};

    team.updateRobots(robot_list);
    team.assignGoalie(0);

    EXPECT_EQ(robot_0, team.goalie());

    team.clearGoalie();

    EXPECT_EQ(std::nullopt, team.goalie());
}

TEST_F(TeamTest, get_goalie_id_with_no_goalie)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    EXPECT_EQ(std::nullopt, team.getGoalieID());
}

TEST_F(TeamTest, get_goalie_id_with_goalie)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);
    team.updateRobots({robot_0});
    team.assignGoalie(0);

    EXPECT_EQ(0, team.getGoalieID());
}

TEST_F(TeamTest, get_robot_expiry_buffer)
{
    Team team = Team(Duration::fromMilliseconds(500));

    EXPECT_EQ(Duration::fromMilliseconds(500), team.getRobotExpiryBufferDuration());
}

TEST_F(TeamTest, set_robot_expiry_buffer)
{
    Team team = Team(Duration::fromMilliseconds(0));

    team.setRobotExpiryBuffer(Duration::fromMilliseconds(831));

    EXPECT_EQ(Duration::fromMilliseconds(831), team.getRobotExpiryBufferDuration());
}

TEST_F(TeamTest, equality_operator_compare_team_with_itself)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Team team = Team(Duration::fromMilliseconds(1000));
    team.updateRobots({robot_0});
    team.assignGoalie(0);

    EXPECT_EQ(Team(Duration::fromMilliseconds(500)),
              Team(Duration::fromMilliseconds(500)));
    EXPECT_EQ(team, team);
}

TEST_F(TeamTest, equality_operator_teams_with_different_expiry_buffers)
{
    EXPECT_NE(Team(Duration::fromMilliseconds(50)),
              Team(Duration::fromMilliseconds(1000)));
}

TEST_F(TeamTest, equality_operator_teams_with_different_number_of_robots)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    Team team_0 = Team(Duration::fromMilliseconds(1000));
    team_0.updateRobots({robot_0, robot_1, robot_2});

    Team team_1 = Team(Duration::fromMilliseconds(1000));
    team_1.updateRobots({robot_0, robot_2});

    EXPECT_NE(team_0, team_1);
}

TEST_F(TeamTest, equality_operator_teams_with_same_number_of_robots_but_different_robots)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    Team team_0 = Team(Duration::fromMilliseconds(1000));
    team_0.updateRobots({robot_0, robot_1});

    Team team_1 = Team(Duration::fromMilliseconds(1000));
    team_1.updateRobots({robot_0, robot_2});

    EXPECT_NE(team_0, team_1);
}

TEST_F(TeamTest, equality_operator_teams_with_different_goalie)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    Team team_0 = Team(Duration::fromMilliseconds(1000));
    team_0.updateRobots({robot_0, robot_1});
    team_0.assignGoalie(0);

    Team team_1 = Team(Duration::fromMilliseconds(1000));
    team_1.updateRobots({robot_0, robot_1});
    team_1.assignGoalie(1);

    EXPECT_NE(team_0, team_1);
}

TEST_F(TeamTest, get_most_recent_timestamp)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), one_second_future);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), one_second_past);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    Team team = Team(Duration::fromMilliseconds(1000));
    team.updateRobots({robot_0, robot_1, robot_2});
    team.assignGoalie(0);

    EXPECT_EQ(one_second_future, team.getMostRecentTimestamp());
}

TEST_F(TeamTest, update_timestamp_history_from_robot_update)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), one_second_future);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), one_second_past);

    Team team = Team(Duration::fromMilliseconds(1000));
    team.updateRobots({robot_0, robot_1});

    robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                    AngularVelocity::threeQuarter(), two_seconds_future);
    robot_1 = Robot(1, Point(0, 1), Vector(-1, -2), Angle::half(),
                    AngularVelocity::threeQuarter(), one_second_future);

    team.updateRobots({robot_0, robot_1});

    robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                    AngularVelocity::threeQuarter(), two_seconds_100ms_future);
    robot_1 = Robot(1, Point(0, 1), Vector(-1, -2), Angle::half(),
                    AngularVelocity::threeQuarter(), two_seconds_100ms_future);

    team.updateRobots({robot_0, robot_1});

    EXPECT_EQ(Timestamp::fromSeconds(0.0), team.getTimestampHistory()[3]);
    EXPECT_EQ(one_second_future, team.getTimestampHistory()[2]);
    EXPECT_EQ(two_seconds_future, team.getTimestampHistory()[1]);
    EXPECT_EQ(two_seconds_100ms_future, team.getTimestampHistory()[0]);
    EXPECT_EQ(4, team.getTimestampHistory().size());
}

TEST_F(TeamTest, update_timestamp_history_from_team_update)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), one_second_future);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), one_second_past);

    Team team = Team(Duration::fromMilliseconds(1000));
    team.updateRobots({robot_0, robot_1});

    robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                    AngularVelocity::threeQuarter(), two_seconds_future);

    robot_1 = Robot(1, Point(0, 1), Vector(-1, -2), Angle::half(),
                    AngularVelocity::threeQuarter(), one_second_future);

    Team team_temp = Team(Duration::fromMilliseconds(1000));
    team_temp.updateRobots({robot_0, robot_1});
    team.updateState(team_temp);

    robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                    AngularVelocity::threeQuarter(), three_seconds_future);

    robot_1 = Robot(1, Point(0, 1), Vector(-1, -2), Angle::half(),
                    AngularVelocity::threeQuarter(), three_seconds_future);
    team_temp.updateRobots({robot_0, robot_1});

    team.updateState(team_temp);
    boost::circular_buffer timestamps = team.getTimestampHistory();

    EXPECT_EQ(Timestamp::fromSeconds(0.0), team.getTimestampHistory()[3]);
    EXPECT_EQ(one_second_future, team.getTimestampHistory()[2]);
    EXPECT_EQ(two_seconds_future, team.getTimestampHistory()[1]);
    EXPECT_EQ(three_seconds_future, team.getTimestampHistory()[0]);
    EXPECT_EQ(4, team.getTimestampHistory().size());
}
