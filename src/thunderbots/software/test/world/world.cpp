#include "ai/world/world.h"
#include <gtest/gtest.h>
#include "test/test_util/util.h"

TEST(WorldTest, default_construction)
{
    World world = World();

    // Check that the basic default values are set
    EXPECT_EQ(Field(), world.field());
    EXPECT_FALSE(world.field().valid());
    EXPECT_EQ(Ball(), world.ball());
    EXPECT_EQ(Point(), world.ball().position());
    EXPECT_EQ(Team(), world.friendly_team());
    EXPECT_EQ(0, world.friendly_team().size());
    EXPECT_EQ(std::nullopt, world.enemy_team().goalie());
}

TEST(WorldTest, construction_with_args)
{
    // Create a field
    Field field = UnitTest::Util::createNormalTestingField();

    // Create a Ball
    Ball ball = Ball();
    ball.update(Point(1, 2), Vector(-0.3, 0));

    // Create a friendly team
    Robot friendly_robot_0 = Robot(0);
    friendly_robot_0.update(Point(0, 1), Vector(-1, -2), Angle::half(),
                            AngularVelocity::threeQuarter());

    Robot friendly_robot_1 = Robot(1);
    friendly_robot_1.update(Point(3, -1), Vector(), Angle::zero(),
                            AngularVelocity::zero());

    Team friendly_team = Team();
    friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});
    friendly_team.updateGoalie(1);

    // Create an enemy team
    Robot enemy_robot_0 = Robot(0);
    enemy_robot_0.update(Point(0.5, -2.5), Vector(), Angle::ofRadians(1),
                         AngularVelocity::ofRadians(2));

    Robot enemy_robot_1 = Robot(1);
    enemy_robot_1.update(Point(), Vector(-0.5, 4), Angle::quarter(),
                         AngularVelocity::half());

    Team enemy_team = Team();
    enemy_team.updateRobots({enemy_robot_0, enemy_robot_1});
    enemy_team.updateGoalie(0);

    // Construct the world with arguments
    World world = World(field, ball, friendly_team, enemy_team);

    // Check that objects used for construction are returned by the accessors
    EXPECT_EQ(field, world.field());
    EXPECT_EQ(ball, world.ball());
    EXPECT_EQ(friendly_team, world.friendly_team());
    EXPECT_EQ(enemy_team, world.enemy_team());

    // Double check that the attributes of the objects are still the same
    EXPECT_TRUE(world.field().valid());
    EXPECT_DOUBLE_EQ(9.0, world.field().length());
    EXPECT_DOUBLE_EQ(1.0, world.field().goalWidth());

    EXPECT_EQ(Point(1, 2), world.ball().position());

    EXPECT_EQ(2, world.friendly_team().size());
    EXPECT_EQ(friendly_robot_1, world.friendly_team().getRobotById(1));

    std::vector<Robot> enemy_robots = {enemy_robot_0, enemy_robot_1};
    EXPECT_EQ(enemy_robots, world.enemy_team().getAllRobots());
    EXPECT_EQ(enemy_robot_0, world.enemy_team().goalie());
}

TEST(WorldTest, update_and_clear)
{
    // Create a field
    Field field = UnitTest::Util::createNormalTestingField();

    // Create a Ball
    Ball ball = Ball();
    ball.update(Point(1, 2), Vector(-0.3, 0));

    // Create a friendly team
    Robot friendly_robot_0 = Robot(0);
    friendly_robot_0.update(Point(0, 1), Vector(-1, -2), Angle::half(),
                            AngularVelocity::threeQuarter());

    Robot friendly_robot_1 = Robot(1);
    friendly_robot_1.update(Point(3, -1), Vector(), Angle::zero(),
                            AngularVelocity::zero());

    Team friendly_team = Team();
    friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});
    friendly_team.updateGoalie(1);

    // Create an enemy team
    Robot enemy_robot_0 = Robot(0);
    enemy_robot_0.update(Point(0.5, -2.5), Vector(), Angle::ofRadians(1),
                         AngularVelocity::ofRadians(2));

    Robot enemy_robot_1 = Robot(1);
    enemy_robot_1.update(Point(), Vector(-0.5, 4), Angle::quarter(),
                         AngularVelocity::half());

    Team enemy_team = Team();
    enemy_team.updateRobots({enemy_robot_0, enemy_robot_1});
    enemy_team.updateGoalie(0);

    // Construct the world using the default constructor
    World world = World();

    // Update the field
    world.updateFieldGeometry(field);
    EXPECT_EQ(field, world.field());
    EXPECT_TRUE(world.field().valid());
    EXPECT_DOUBLE_EQ(6.6, world.field().totalWidth());
    EXPECT_DOUBLE_EQ(1.0, world.field().defenseAreaLength());

    // Update the ball
    world.updateBallState(ball);
    EXPECT_EQ(ball, world.ball());
    EXPECT_EQ(Vector(-0.3, 0), world.ball().velocity());

    // Update the friendly team
    world.updateFriendlyTeam(friendly_team);
    EXPECT_EQ(friendly_team, world.friendly_team());
    EXPECT_EQ(2, world.friendly_team().size());
    EXPECT_EQ(friendly_robot_1, world.friendly_team().getRobotById(1));

    // Update the enemy team
    world.updateEnemyTeam(enemy_team);
    EXPECT_EQ(enemy_team, world.enemy_team());
    std::vector<Robot> enemy_robots = {enemy_robot_0, enemy_robot_1};
    EXPECT_EQ(enemy_robots, world.enemy_team().getAllRobots());
    EXPECT_EQ(enemy_robot_0, world.enemy_team().goalie());

    // Make sure that the updates haven't affected the other values
    EXPECT_EQ(field, world.field());
    EXPECT_EQ(ball, world.ball());
    EXPECT_EQ(friendly_team, world.friendly_team());
    EXPECT_EQ(enemy_team, world.enemy_team());

    // Clear friendly robots
    world.clearFriendlyTeamRobots();
    EXPECT_NE(friendly_team, world.friendly_team());
    EXPECT_EQ(0, world.friendly_team().size());
    EXPECT_EQ(std::nullopt, world.friendly_team().getRobotById(1));
    EXPECT_EQ(std::nullopt, world.friendly_team().goalie());

    // Clear enemy robots
    world.clearEnemyTeamRobots();
    EXPECT_NE(enemy_team, world.enemy_team());
    EXPECT_EQ(0, world.enemy_team().size());
    EXPECT_EQ(std::nullopt, world.enemy_team().getRobotById(0));
    EXPECT_EQ(std::nullopt, world.enemy_team().goalie());

    // Make sure we can properly update teams again after clearing robots
    world.updateEnemyTeam(enemy_team);
    EXPECT_EQ(enemy_team, world.enemy_team());
    EXPECT_EQ(enemy_robots, world.enemy_team().getAllRobots());
    EXPECT_EQ(enemy_robot_0, world.enemy_team().goalie());
    EXPECT_EQ(enemy_robot_1, world.enemy_team().getRobotById(1));
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
