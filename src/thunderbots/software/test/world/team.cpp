#include "ai/world/team.h"
#include <gtest/gtest.h>

using namespace std::chrono;

class TeamTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        auto epoch       = time_point<std::chrono::steady_clock>();
        auto since_epoch = std::chrono::seconds(10000);

        // An arbitrary fixed point in time. 10000 seconds after the epoch.
        // We use this fixed point in time to make the tests deterministic.
        current_time             = epoch + since_epoch;
        one_second_future        = current_time + seconds(1);
        two_seconds_future       = current_time + seconds(2);
        two_seconds_100ms_future = current_time + milliseconds(2100);
        three_seconds_future     = current_time + seconds(3);
        four_seconds_future      = current_time + seconds(4);
        five_seconds_future      = current_time + seconds(5);
    }

    steady_clock::time_point current_time;
    steady_clock::time_point one_second_future;
    steady_clock::time_point two_seconds_future;
    steady_clock::time_point two_seconds_100ms_future;
    steady_clock::time_point three_seconds_future;
    steady_clock::time_point four_seconds_future;
    steady_clock::time_point five_seconds_future;
};

TEST_F(TeamTest, construction)
{
    Team team = Team(milliseconds(1000));

    EXPECT_EQ(0, team.size());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(std::vector<Robot>(), team.getAllRobots());
    EXPECT_EQ(milliseconds(1000), team.getRobotExpiryBufferMilliseconds());
}

TEST_F(TeamTest, update_with_robots)
{
    Team team = Team(milliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1, robot_2};

    team.updateRobots(robot_list, current_time);

    EXPECT_EQ(3, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());
}

TEST_F(TeamTest, update_with_new_team)
{
    Team team_update = Team(milliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1, robot_2};

    team_update.updateRobots(robot_list, current_time);

    Team team = Team(milliseconds(1000));

    team.updateState(team_update, current_time);

    EXPECT_EQ(3, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());

    EXPECT_EQ(team_update, team);
}

TEST_F(TeamTest, update_state_to_predicted_state_with_future_timestamp)
{
    Team team = Team(milliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1};

    team.updateRobots(robot_list, current_time);
    team.assignGoalie(0);

    team.updateStateToPredictedState(one_second_future);

    Robot future_robot_0 = robot_0;
    future_robot_0.updateStateToPredictedState(one_second_future);

    Robot future_robot_1 = robot_1;
    future_robot_1.updateStateToPredictedState(one_second_future);

    EXPECT_EQ(2, team.size());
    EXPECT_EQ(future_robot_0, team.getRobotById(0));
    EXPECT_EQ(future_robot_1, team.getRobotById(1));
    EXPECT_EQ(future_robot_0, team.goalie());
}

TEST_F(TeamTest, remove_expired_robots)
{
    Team team = Team(milliseconds(2150));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), one_second_future);

    team.updateRobots({robot_0, robot_1}, current_time);

    team.removeExpiredRobots(current_time);

    EXPECT_EQ(2, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));

    team.removeExpiredRobots(two_seconds_100ms_future);

    EXPECT_EQ(2, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));

    team.removeExpiredRobots(three_seconds_future);

    EXPECT_EQ(1, team.size());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));

    team.removeExpiredRobots(four_seconds_future);

    EXPECT_EQ(0, team.size());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
}

TEST_F(TeamTest, update_robots_no_robots_expire)
{
    Team team = Team(milliseconds(5000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    // Add a single robot to the team
    team.updateRobots({robot_0}, current_time);

    EXPECT_EQ(1, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));

    // Pretend it has been 1 second since the last update, with no new updates
    team.updateRobots({}, one_second_future);

    EXPECT_EQ(1, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));

    // Pretend it has been 4 seconds since the last update, with no new updates
    team.updateRobots({}, four_seconds_future);

    EXPECT_EQ(1, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
}

TEST_F(TeamTest, update_robots_all_robots_expire)
{
    Team team = Team(milliseconds(2000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    // Add a single robot to the team
    team.updateRobots({robot_0}, current_time);

    EXPECT_EQ(1, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));

    // Update again with no new data. Pretend it has been 2.1 seconds since the
    // robot was last updated. This is past the debounce threshold, so we expect the
    // robot to have expired and been removed from the team
    team.updateRobots({}, two_seconds_100ms_future);

    EXPECT_EQ(0, team.size());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
}

TEST_F(TeamTest, update_robots_staggered_robot_expiry)
{
    Team team = Team(milliseconds(2500));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    team.updateRobots({robot_0}, current_time);

    EXPECT_EQ(1, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));

    // Add a robot 1 second later. The first robot has not expired yet
    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), one_second_future);

    team.updateRobots({robot_1}, one_second_future);

    EXPECT_EQ(2, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));

    // Add a robot another second later. The first 2 robots have not expired yet

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), two_seconds_future);

    team.updateRobots({robot_2}, two_seconds_future);

    EXPECT_EQ(3, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));

    // Update with no new data. It has now been 3 seconds since the first robot was added,
    // which is past the debounce threshold. Therefore we expect the first robot to have
    // been removed from the team
    team.updateRobots({}, three_seconds_future);

    EXPECT_EQ(2, team.size());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));

    // Update again. The second robot should have expired now
    team.updateRobots({}, four_seconds_future);

    EXPECT_EQ(1, team.size());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));

    // Update again. The third robot should have expired now
    team.updateRobots({}, five_seconds_future);

    EXPECT_EQ(0, team.size());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));
}

TEST_F(TeamTest, clear_all_robots)
{
    Team team = Team(milliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1};

    team.updateRobots(robot_list, current_time);

    EXPECT_EQ(2, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());

    team.clearAllRobots();

    EXPECT_EQ(0, team.size());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(std::vector<Robot>(), team.getAllRobots());
}

TEST_F(TeamTest, assign_and_clear_goalie)
{
    Team team = Team(milliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1};

    team.updateRobots(robot_list, current_time);

    EXPECT_EQ(std::nullopt, team.goalie());

    team.clearGoalie();

    EXPECT_EQ(std::nullopt, team.goalie());

    team.assignGoalie(0);

    EXPECT_EQ(robot_0, team.goalie());

    team.clearGoalie();

    EXPECT_EQ(std::nullopt, team.goalie());
}

TEST_F(TeamTest, equality_operators)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    Team team_0 = Team(milliseconds(1000));
    team_0.updateRobots({robot_0, robot_2}, current_time);
    team_0.assignGoalie(0);

    Team team_1 = Team(milliseconds(1000));
    team_1.updateRobots({robot_0, robot_2}, current_time);
    team_1.assignGoalie(2);

    Team team_2 = Team(milliseconds(1000));
    team_2.updateRobots({robot_0, robot_1, robot_2}, current_time);
    team_2.assignGoalie(0);

    EXPECT_EQ(Team(milliseconds(1)), Team(milliseconds(1)));
    EXPECT_EQ(team_0, team_0);
    EXPECT_EQ(team_2, team_2);

    EXPECT_NE(team_0, team_1);
    EXPECT_NE(team_0, team_2);
    EXPECT_NE(team_1, team_2);

    // Now that team_0 and team_1 have the same goalie (they already had the same robots)
    // they are equal. Even though team_1 was created/updated after team_0, the different
    // timestamps are not considered for equality
    team_0.assignGoalie(2);
    EXPECT_EQ(team_0, team_1);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
