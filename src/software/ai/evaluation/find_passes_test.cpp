#include "software/ai/evaluation/find_passes.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/test_util/test_util.h"

TEST(FindPasses, find_all_passes_with_no_enemies)
{
    Team friendly_team = Team(Duration::fromSeconds(1));
    Team enemy_team    = Team(Duration::fromSeconds(1));

    Robot initial_robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_1 = Robot(1, Point(2, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));


    Robot friendly_robot_2 = Robot(2, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_3 = Robot(3, Point(0, 2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_4 = Robot(4, Point(0, -2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    friendly_team.updateRobots({initial_robot, friendly_robot_1, friendly_robot_2,
                                friendly_robot_3, friendly_robot_4});

    AllPasses all_passes = findAllPasses(initial_robot, friendly_team, enemy_team);
    std::vector<Robot> expected_direct_passes   = {friendly_robot_1, friendly_robot_2,
                                                 friendly_robot_3, friendly_robot_4};
    std::vector<Robot> expected_indirect_passes = {};
    EXPECT_EQ(expected_direct_passes, all_passes.direct_passes);
    EXPECT_EQ(expected_indirect_passes, all_passes.indirect_passes);
}

TEST(FindPasses, find_all_passes_with_one_enemy)
{
    Team friendly_team = Team(Duration::fromSeconds(1));
    Team enemy_team    = Team(Duration::fromSeconds(1));

    Robot initial_robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_1 = Robot(1, Point(2, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));


    Robot friendly_robot_2 = Robot(2, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_3 = Robot(3, Point(0, 2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_4 = Robot(4, Point(0, -2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot enemy_robot_0 = Robot(0, Point(1, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    friendly_team.updateRobots({initial_robot, friendly_robot_1, friendly_robot_2,
                                friendly_robot_3, friendly_robot_4});
    enemy_team.updateRobots({enemy_robot_0});
    AllPasses all_passes = findAllPasses(initial_robot, friendly_team, enemy_team);

    std::vector<Robot> expected_direct_passes   = {friendly_robot_2, friendly_robot_3,
                                                 friendly_robot_4};
    std::vector<Robot> expected_indirect_passes = {friendly_robot_1};
    EXPECT_EQ(expected_direct_passes, all_passes.direct_passes);
    EXPECT_EQ(expected_indirect_passes, all_passes.indirect_passes);
}

TEST(FindPasses, find_all_passes_with_one_enemy_not_directly_in_path)
{
    Team friendly_team = Team(Duration::fromSeconds(1));
    Team enemy_team    = Team(Duration::fromSeconds(1));

    Robot initial_robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_1 = Robot(1, Point(2, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));


    Robot friendly_robot_2 = Robot(2, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_3 = Robot(3, Point(0, 2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_4 = Robot(4, Point(0, -2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot enemy_robot_0 = Robot(0, Point(0.99, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    friendly_team.updateRobots({initial_robot, friendly_robot_1, friendly_robot_2,
                                friendly_robot_3, friendly_robot_4});
    enemy_team.updateRobots({enemy_robot_0});
    AllPasses all_passes = findAllPasses(initial_robot, friendly_team, enemy_team);


    std::vector<Robot> expected_direct_passes   = {friendly_robot_2, friendly_robot_3,
                                                 friendly_robot_4};
    std::vector<Robot> expected_indirect_passes = {friendly_robot_1};
    EXPECT_EQ(expected_direct_passes, all_passes.direct_passes);
    EXPECT_EQ(expected_indirect_passes, all_passes.indirect_passes);
}


TEST(FindPasses, find_no_direct_passes)
{
    Team friendly_team = Team(Duration::fromSeconds(1));
    Team enemy_team    = Team(Duration::fromSeconds(1));

    Robot initial_robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_1 = Robot(1, Point(2, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));


    Robot friendly_robot_2 = Robot(2, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_3 = Robot(3, Point(0, 2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_4 = Robot(4, Point(0, -2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot enemy_robot_0 = Robot(0, Point(1, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_1 = Robot(1, Point(-1, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot enemy_robot_2 = Robot(2, Point(0, 1), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot enemy_robot_3 = Robot(3, Point(0, -1), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    friendly_team.updateRobots({initial_robot, friendly_robot_1, friendly_robot_2,
                                friendly_robot_3, friendly_robot_4});
    enemy_team.updateRobots({enemy_robot_0, enemy_robot_1, enemy_robot_2, enemy_robot_3});
    AllPasses all_passes = findAllPasses(initial_robot, friendly_team, enemy_team);

    std::vector<Robot> expected_direct_passes   = {};
    std::vector<Robot> expected_indirect_passes = {friendly_robot_1, friendly_robot_2,
                                                   friendly_robot_3, friendly_robot_4};
    EXPECT_EQ(expected_direct_passes, all_passes.direct_passes);
    EXPECT_EQ(expected_indirect_passes, all_passes.indirect_passes);
}


TEST(FindPasses, find_passes_with_no_open_robots)
{
    Team friendly_team = Team(Duration::fromSeconds(1));
    Team enemy_team    = Team(Duration::fromSeconds(1));

    Robot initial_robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_1 = Robot(1, Point(2, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));


    Robot friendly_robot_2 = Robot(2, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_3 = Robot(3, Point(0, 2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_4 = Robot(4, Point(0, -2), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot enemy_robot_0 = Robot(0, Point(2.01, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_robot_1 = Robot(1, Point(-2.01, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot enemy_robot_2 = Robot(2, Point(0, 2.01), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot enemy_robot_3 = Robot(3, Point(0, -2.01), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    friendly_team.updateRobots({initial_robot, friendly_robot_1, friendly_robot_2,
                                friendly_robot_3, friendly_robot_4});
    enemy_team.updateRobots({enemy_robot_0, enemy_robot_1, enemy_robot_2, enemy_robot_3});
    AllPasses all_passes = findAllPasses(initial_robot, friendly_team, enemy_team);

    std::vector<Robot> expected_direct_passes   = {};
    std::vector<Robot> expected_indirect_passes = {};

    EXPECT_EQ(expected_direct_passes, all_passes.direct_passes);
    EXPECT_EQ(expected_indirect_passes, all_passes.indirect_passes);
}



TEST(FindPasses, find_passes_with_friendly_robots_blocking_passes)
{
    Team friendly_team = Team(Duration::fromSeconds(1));
    Team enemy_team    = Team(Duration::fromSeconds(1));

    Robot initial_robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_1 = Robot(1, Point(1, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));


    Robot friendly_robot_2 = Robot(2, Point(2, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot friendly_robot_3 = Robot(3, Point(0, -1), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    friendly_team.updateRobots(
        {initial_robot, friendly_robot_1, friendly_robot_2, friendly_robot_3});
    AllPasses all_passes = findAllPasses(initial_robot, friendly_team, enemy_team);

    std::vector<Robot> expected_direct_passes   = {friendly_robot_1, friendly_robot_3};
    std::vector<Robot> expected_indirect_passes = {friendly_robot_2};

    EXPECT_EQ(expected_direct_passes, all_passes.direct_passes);
    EXPECT_EQ(expected_indirect_passes, all_passes.indirect_passes);
}
