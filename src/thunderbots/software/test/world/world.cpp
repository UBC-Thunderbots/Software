#include "ai/world/world.h"

#include <gtest/gtest.h>

#include "test/test_util/test_util.h"

class WorldTest : public ::testing::Test
{
   protected:
    WorldTest() : ::testing::Test(), current_time(Timestamp::fromSeconds(0)) {}

    void SetUp() override
    {
        // An arbitrary fixed point in time
        // We use this fixed point in time to make the tests deterministic.
        current_time = Timestamp::fromSeconds(123);
    }

    Timestamp current_time;
};

TEST_F(WorldTest, construction_with_parameters)
{
    Field field = ::Test::TestUtil::createSSLDivBField();

    Ball ball = Ball(Point(1, 2), Vector(-0.3, 0), current_time);

    Robot friendly_robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                                   AngularVelocity::threeQuarter(), current_time);

    Robot friendly_robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                                   AngularVelocity::zero(), current_time);

    Team friendly_team = Team(Duration::fromMilliseconds(1000));
    friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});
    friendly_team.assignGoalie(1);

    Robot enemy_robot_0 = Robot(0, Point(0.5, -2.5), Vector(), Angle::ofRadians(1),
                                AngularVelocity::ofRadians(2), current_time);

    Robot enemy_robot_1 = Robot(1, Point(), Vector(-0.5, 4), Angle::quarter(),
                                AngularVelocity::half(), current_time);

    Team enemy_team = Team(Duration::fromMilliseconds(1000));
    enemy_team.updateRobots({enemy_robot_0, enemy_robot_1});
    enemy_team.assignGoalie(0);

    // Construct the world with arguments
    World world = World(field, ball, friendly_team, enemy_team);

    // Check that objects used for construction are returned by the accessors
    EXPECT_EQ(field, world.field());
    EXPECT_EQ(ball, world.ball());
    EXPECT_EQ(friendly_team, world.friendlyTeam());
    EXPECT_EQ(enemy_team, world.enemyTeam());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
