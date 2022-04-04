#include "software/ai/evaluation/find_passes.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/test_util/test_util.h"

TEST(FindAllDirectPasses, find_all_direct_passes_with_no_enemies)
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
    // world.updateFriendlyTeamState(team);
    std::vector<Pass> passes = findDirectPasses(initial_robot, friendly_team, enemy_team);


    std::vector<Pass> expected_passes = {
        Pass(initial_robot.position(), friendly_robot_1.position(),
             BALL_MAX_SPEED_METERS_PER_SECOND),
        Pass(initial_robot.position(), friendly_robot_2.position(),
             BALL_MAX_SPEED_METERS_PER_SECOND),
        Pass(initial_robot.position(), friendly_robot_3.position(),
             BALL_MAX_SPEED_METERS_PER_SECOND),
        Pass(initial_robot.position(), friendly_robot_4.position(),
             BALL_MAX_SPEED_METERS_PER_SECOND)};
    EXPECT_EQ(expected_passes, passes);
}
