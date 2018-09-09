#include <gtest/gtest.h>
#include "test/test_util/util.h"

TEST(TestUtilsTest, field_creation)
{
    Field field = UnitTest::Util::createNormalTestingField();

    // Check that the field is valid and has the correct dimensions for a
    // SSL Division B field according to the rules
    EXPECT_TRUE(field.valid());

    EXPECT_DOUBLE_EQ(9.6, field.totalLength());
    EXPECT_DOUBLE_EQ(6.6, field.totalWidth());
    EXPECT_DOUBLE_EQ(0.3, field.boundaryWidth());

    EXPECT_EQ(Point(-4.5, 0.0), field.friendlyGoal());
    EXPECT_EQ(Point(4.5, 0.0), field.enemyGoal());

    EXPECT_EQ(Point(-4.5, 0.5), field.friendlyGoalpostPos());
    EXPECT_EQ(Point(-4.5, -0.5), field.friendlyGoalpostNeg());
    EXPECT_EQ(Point(4.5, 0.5), field.enemyGoalpostPos());
    EXPECT_EQ(Point(4.5, -0.5), field.enemyGoalpostNeg());

    EXPECT_EQ(Rect(Point(-4.5, 1.0), Point(-3.5, -1.0)), field.friendlyDefenseArea());
    EXPECT_EQ(Rect(Point(4.5, 1.0), Point(3.5, -1.0)), field.enemyDefenseArea());

    EXPECT_EQ(Point(-3.5, 0.0), field.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 3.0), field.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -3.0), field.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 3.0), field.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -3.0), field.enemyCornerNeg());
}

TEST(TestUtilsTest, world_creation)
{
    World world = UnitTest::Util::createNormalTestingWorld();

    // Test that the field is a "normal testing field", with the dimensions of
    // a SSL Division B field as specified in the rules
    Field field = world.field();

    EXPECT_TRUE(field.valid());

    EXPECT_DOUBLE_EQ(9.6, field.totalLength());
    EXPECT_DOUBLE_EQ(6.6, field.totalWidth());
    EXPECT_DOUBLE_EQ(0.3, field.boundaryWidth());

    EXPECT_EQ(Point(-4.5, 0.0), field.friendlyGoal());
    EXPECT_EQ(Point(4.5, 0.0), field.enemyGoal());

    EXPECT_EQ(Point(-4.5, 0.5), field.friendlyGoalpostPos());
    EXPECT_EQ(Point(-4.5, -0.5), field.friendlyGoalpostNeg());
    EXPECT_EQ(Point(4.5, 0.5), field.enemyGoalpostPos());
    EXPECT_EQ(Point(4.5, -0.5), field.enemyGoalpostNeg());

    EXPECT_EQ(Rect(Point(-4.5, 1.0), Point(-3.5, -1.0)), field.friendlyDefenseArea());
    EXPECT_EQ(Rect(Point(4.5, 1.0), Point(3.5, -1.0)), field.enemyDefenseArea());

    EXPECT_EQ(Point(-3.5, 0.0), field.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 3.0), field.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -3.0), field.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 3.0), field.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -3.0), field.enemyCornerNeg());

    // Test the Ball is initialized with default values
    Ball ball = world.ball();
    EXPECT_EQ(Point(), ball.position());
    EXPECT_EQ(Point(), ball.velocity());

    // Test the friendly team is initialized with default values (no robots)
    Team friendly_team = world.friendly_team();
    EXPECT_EQ(0, friendly_team.size());
    EXPECT_EQ(std::nullopt, friendly_team.goalie());
    EXPECT_EQ(std::vector<Robot>(), friendly_team.getAllRobots());

    // Test the enemy team is initialized with default values (no robots)
    Team enemy_team = world.friendly_team();
    EXPECT_EQ(0, enemy_team.size());
    EXPECT_EQ(std::nullopt, enemy_team.goalie());
    EXPECT_EQ(std::vector<Robot>(), enemy_team.getAllRobots());
}

TEST(TestUtilsTest, set_robot_positions)
{
    World world = UnitTest::Util::createNormalTestingWorld();

    world = UnitTest::Util::setFriendlyRobotPositions(
        world, {Point(), Point(-4, 1.2), Point(2.2, -0.1)});

    EXPECT_EQ(3, world.friendly_team().size());
    EXPECT_EQ(0, world.enemy_team().size());
    EXPECT_EQ(Point(), (*world.friendly_team().getRobotById(0)).position());
    EXPECT_EQ(Point(-4, 1.2), (*world.friendly_team().getRobotById(1)).position());
    EXPECT_EQ(Point(2.2, -0.1), (*world.friendly_team().getRobotById(2)).position());
    EXPECT_EQ(std::nullopt, world.friendly_team().getRobotById(3));

    world = UnitTest::Util::setFriendlyRobotPositions(world, {Point(1, 1)});

    EXPECT_EQ(1, world.friendly_team().size());
    EXPECT_EQ(Point(1, 1), (*world.friendly_team().getRobotById(0)).position());
    EXPECT_EQ(std::nullopt, world.friendly_team().getRobotById(1));
    EXPECT_EQ(std::nullopt, world.friendly_team().getRobotById(2));

    world = UnitTest::Util::setEnemyRobotPositions(
        world, {world.field().enemyGoal(), world.field().friendlyCornerPos()});

    EXPECT_EQ(2, world.enemy_team().size());
    EXPECT_EQ(world.field().enemyGoal(),
              (*world.enemy_team().getRobotById(0)).position());
    EXPECT_EQ(world.field().friendlyCornerPos(),
              (*world.enemy_team().getRobotById(1)).position());
    EXPECT_EQ(Vector(), (*world.enemy_team().getRobotById(0)).velocity());
    EXPECT_EQ(Angle::zero(), (*world.enemy_team().getRobotById(0)).orientation());

    // Make sure that setting the enemy robot positions has not affected the friendly
    // robots
    EXPECT_EQ(1, world.friendly_team().size());
    EXPECT_EQ(Point(1, 1), (*world.friendly_team().getRobotById(0)).position());
    EXPECT_EQ(std::nullopt, world.friendly_team().getRobotById(1));
    EXPECT_EQ(std::nullopt, world.friendly_team().getRobotById(2));

    world = UnitTest::Util::setEnemyRobotPositions(world, {});
    EXPECT_EQ(0, world.enemy_team().size());
    EXPECT_EQ(std::nullopt, world.enemy_team().getRobotById(0));
    EXPECT_EQ(std::nullopt, world.enemy_team().getRobotById(1));
}

TEST(TestUtilsTest, set_ball_position_and_velocity)
{
    World world = UnitTest::Util::createNormalTestingWorld();

    world = UnitTest::Util::setBallPosition(world, Point(1, 1));
    EXPECT_EQ(Point(1, 1), world.ball().position());
    EXPECT_EQ(Vector(), world.ball().velocity());

    world = UnitTest::Util::setBallPosition(world, Point(-0.2, 3.11));
    EXPECT_EQ(Point(-0.2, 3.11), world.ball().position());
    EXPECT_EQ(Vector(), world.ball().velocity());

    world = UnitTest::Util::setBallVelocity(world, Vector(0, -2));
    EXPECT_EQ(Point(-0.2, 3.11), world.ball().position());
    EXPECT_EQ(Vector(0, -2), world.ball().velocity());

    world = UnitTest::Util::setBallVelocity(world, Vector());
    EXPECT_EQ(Point(-0.2, 3.11), world.ball().position());
    EXPECT_EQ(Vector(), world.ball().velocity());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
