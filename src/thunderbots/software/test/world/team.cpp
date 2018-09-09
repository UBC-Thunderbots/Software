#include "ai/world/team.h"
#include <gtest/gtest.h>
#include <util/parameter/dynamic_parameters.h>
#include <chrono>
#include <thread>

TEST(TeamTest, construction)
{
    Team team = Team();

    EXPECT_EQ(0, team.size());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(std::vector<Robot>(), team.getAllRobots());
}

TEST(TeamTest, update_and_accessors)
{
    Team team = Team();

    Robot robot_0 = Robot(0);
    robot_0.update(Point(0, 1), Vector(-1, -2), Angle::half(),
                   AngularVelocity::threeQuarter());

    Robot robot_1 = Robot(1);
    robot_1.update(Point(3, -1), Vector(), Angle::zero(), AngularVelocity::zero());

    Robot robot_2 = Robot(2);
    robot_2.update(Point(), Vector(-0.5, 4), Angle::quarter(), AngularVelocity::half());

    std::vector<Robot> robot_list = {robot_0, robot_1, robot_2};

    team.updateRobots(robot_list);

    EXPECT_EQ(3, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());

    team.updateGoalie(0);
    EXPECT_EQ(robot_0, team.goalie());

    team.updateGoalie(2);
    EXPECT_EQ(robot_2, team.goalie());

    team.updateGoalie(5);
    EXPECT_EQ(std::nullopt, team.goalie());
}

TEST(TeamTest, update_with_delay)
{
    DynamicParameters::robot_vision_debounce_milliseconds.setValueLocally(100);

    Team team = Team();

    Robot robot_0 = Robot(0);
    robot_0.update(Point(0, 1), Vector(-1, -2), Angle::half(),
                   AngularVelocity::threeQuarter());

    Robot robot_1 = Robot(1);
    robot_1.update(Point(3, -1), Vector(), Angle::zero(), AngularVelocity::zero());

    Robot robot_2 = Robot(2);
    robot_2.update(Point(), Vector(-0.5, 4), Angle::quarter(), AngularVelocity::half());

    // Add a single robot to the team
    team.updateRobots({robot_0});

    EXPECT_EQ(1, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));

    // Add another robot after the first one. The first robot should still be part of the
    // team since this is happening immediately after
    team.updateRobots({robot_1});

    EXPECT_EQ(2, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));

    // Sleep for half the robot debounce time. This simulates a gap in the camera data
    // that updates the robots, for example if a robot is briefly covered by something.
    std::this_thread::sleep_for(std::chrono::milliseconds(
        DynamicParameters::robot_vision_debounce_milliseconds.value() / 2));

    // As we have not exceeded the debounce time since the robots were last updated, we
    // still expect to have the first 2 robots on the team.
    EXPECT_EQ(2, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));

    // Update all the robots. This refreshes the timestamps for robots 0 and 1
    team.updateRobots({robot_0, robot_1, robot_2});

    EXPECT_EQ(3, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));

    // Sleep for 3/4 of the debounce time.
    std::this_thread::sleep_for(std::chrono::milliseconds(
        DynamicParameters::robot_vision_debounce_milliseconds.value() * 3 / 4));

    // Despite more than the debounce time having passed since robots 0 and 1 were
    // initially added to the team, they still exist because they were updated 3/4 of the
    // debounce time ago, so their timestamps were refreshed and they have not expired.
    EXPECT_EQ(3, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));

    // Sleep for 1/2 of the debounce time.
    std::this_thread::sleep_for(std::chrono::milliseconds(
        DynamicParameters::robot_vision_debounce_milliseconds.value() / 2));

    // Simulate an update with no data, for example if none of the robots are visible
    // on the cameras
    team.updateRobots({});

    // Now more than the debounce time has passed since any of the robots were last
    // updated, so they expire and are removed from the team since they are no longer
    // likely in play / on the field.
    EXPECT_EQ(0, team.size());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));
}

TEST(TeamTest, clear_all_robots)
{
    Team team = Team();

    Robot robot_0 = Robot(0);
    robot_0.update(Point(0, 1), Vector(-1, -2), Angle::half(),
                   AngularVelocity::threeQuarter());

    Robot robot_1 = Robot(1);
    robot_0.update(Point(3, -1), Vector(), Angle::zero(), AngularVelocity::zero());

    Robot robot_2 = Robot(2);
    robot_0.update(Point(), Vector(-0.5, 4), Angle::quarter(), AngularVelocity::half());

    std::vector<Robot> robot_list = {robot_0, robot_1, robot_2};

    team.updateRobots(robot_list);

    EXPECT_EQ(3, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());

    team.updateGoalie(0);
    EXPECT_EQ(robot_0, team.goalie());

    team.clearAllRobots();

    EXPECT_EQ(0, team.size());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(std::vector<Robot>(), team.getAllRobots());

    team.updateRobots({robot_0});
    EXPECT_EQ(1, team.size());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(robot_0, team.goalie());
    EXPECT_EQ(std::vector<Robot>{robot_0}, team.getAllRobots());

    team.clearAllRobots();
    EXPECT_EQ(0, team.size());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(std::vector<Robot>(), team.getAllRobots());
}

TEST(TeamTest, equality_operators)
{
    Robot robot_0 = Robot(0);
    robot_0.update(Point(0, 1), Vector(-1, -2), Angle::half(),
                   AngularVelocity::threeQuarter());

    Robot robot_1 = Robot(1);
    robot_1.update(Point(3, -1), Vector(), Angle::zero(), AngularVelocity::zero());

    Robot robot_2 = Robot(2);
    robot_2.update(Point(), Vector(-0.5, 4), Angle::quarter(), AngularVelocity::half());

    Team team_0 = Team();
    team_0.updateRobots({robot_0, robot_2});
    team_0.updateGoalie(0);

    Team team_1 = Team();
    team_1.updateRobots({robot_0, robot_2});
    team_1.updateGoalie(2);

    Team team_2 = Team();
    team_2.updateRobots({robot_0, robot_1, robot_2});
    team_2.updateGoalie(0);

    // A team should be equal to itself (reflexive)
    EXPECT_EQ(Team(), Team());
    EXPECT_EQ(team_0, team_0);
    EXPECT_EQ(team_2, team_2);

    EXPECT_NE(team_0, team_1);
    EXPECT_NE(team_0, team_2);
    EXPECT_NE(team_1, team_2);

    // Now that team_0 and team_1 have the same goalie (they already had the same robots)
    // they are equal. Even though team_1 was created/updated after team_0, the different
    // timestamps are not considered for equality
    team_0.updateGoalie(2);
    EXPECT_EQ(team_0, team_1);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
