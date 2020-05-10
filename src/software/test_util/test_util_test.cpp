#include "software/test_util/test_util.h"

#include <gtest/gtest.h>

#include <algorithm>

/*
 * Unit tests for the unit test utilities
 */
TEST(TestUtilsTest, create_testing_field)
{
    Field field = ::Test::TestUtil::createSSLDivBField();

    // Check that the field has the correct dimensions for a
    // SSL Division B field according to the rules
    EXPECT_DOUBLE_EQ(9.6, field.totalXLength());
    EXPECT_DOUBLE_EQ(6.6, field.totalYLength());
    EXPECT_DOUBLE_EQ(0.3, field.boundaryYLength());

    EXPECT_EQ(Point(-4.5, 0.0), field.friendlyGoal());
    EXPECT_EQ(Point(4.5, 0.0), field.enemyGoal());

    EXPECT_EQ(Point(-4.5, 0.5), field.friendlyGoalpostPos());
    EXPECT_EQ(Point(-4.5, -0.5), field.friendlyGoalpostNeg());
    EXPECT_EQ(Point(4.5, 0.5), field.enemyGoalpostPos());
    EXPECT_EQ(Point(4.5, -0.5), field.enemyGoalpostNeg());

    EXPECT_EQ(Rectangle(Point(-4.5, 1.0), Point(-3.5, -1.0)),
              field.friendlyDefenseArea());
    EXPECT_EQ(Rectangle(Point(4.5, 1.0), Point(3.5, -1.0)), field.enemyDefenseArea());

    EXPECT_EQ(Point(-3.5, 0.0), field.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 3.0), field.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -3.0), field.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 3.0), field.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -3.0), field.enemyCornerNeg());
}

TEST(TestUtilsTest, create_testing_world)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    EXPECT_EQ(::Test::TestUtil::createSSLDivBField(), world.field());
    EXPECT_EQ(Team(Duration::fromMilliseconds(1000)), world.friendlyTeam());
    EXPECT_EQ(Team(Duration::fromMilliseconds(1000)), world.enemyTeam());
    EXPECT_EQ(Ball(Point(), Vector(), Timestamp::fromSeconds(0)), world.ball());
}

TEST(TestUtilsTest, set_friendly_robot_positions_in_world_with_positive_number_of_robots)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    world = ::Test::TestUtil::setFriendlyRobotPositions(
        world, {Point(), Point(-4, 1.2), Point(2.2, -0.1)}, Timestamp::fromSeconds(0));

    EXPECT_EQ(3, world.friendlyTeam().numRobots());
    EXPECT_EQ(0, world.enemyTeam().numRobots());
    EXPECT_EQ(Point(), (*world.friendlyTeam().getRobotById(0)).position());
    EXPECT_EQ(Point(-4, 1.2), (*world.friendlyTeam().getRobotById(1)).position());
    EXPECT_EQ(Point(2.2, -0.1), (*world.friendlyTeam().getRobotById(2)).position());
    EXPECT_EQ(std::nullopt, world.friendlyTeam().getRobotById(3));
}

TEST(TestUtilsTest, set_enemy_robot_positions_in_world_with_positive_number_of_robots)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    world = ::Test::TestUtil::setEnemyRobotPositions(
        world, {world.field().enemyGoal(), world.field().friendlyCornerPos()},
        Timestamp::fromSeconds(0));

    EXPECT_EQ(2, world.enemyTeam().numRobots());
    EXPECT_EQ(world.field().enemyGoal(), (*world.enemyTeam().getRobotById(0)).position());
    EXPECT_EQ(world.field().friendlyCornerPos(),
              (*world.enemyTeam().getRobotById(1)).position());
    EXPECT_EQ(Vector(), (*world.enemyTeam().getRobotById(0)).velocity());
    EXPECT_EQ(Angle::zero(), (*world.enemyTeam().getRobotById(0)).orientation());
}

TEST(TestUtilsTest, set_friendly_robot_positions_in_world_with_zero_robots)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    world =
        ::Test::TestUtil::setFriendlyRobotPositions(world, {}, Timestamp::fromSeconds(0));

    EXPECT_EQ(0, world.friendlyTeam().numRobots());
}

TEST(TestUtilsTest, set_enemy_robot_positions_in_world_with_zero_robots)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    world =
        ::Test::TestUtil::setEnemyRobotPositions(world, {}, Timestamp::fromSeconds(0));

    EXPECT_EQ(0, world.enemyTeam().numRobots());
}

TEST(TestUtilsTest, set_ball_position_in_world)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    world = ::Test::TestUtil::setBallPosition(world, Point(-0.2, 3.11),
                                              Timestamp::fromSeconds(0));
    EXPECT_EQ(Point(-0.2, 3.11), world.ball().position());
    EXPECT_EQ(Vector(), world.ball().velocity());
}

TEST(TestUtilsTest, set_ball_velocity_in_world)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    world = ::Test::TestUtil::setBallVelocity(world, Vector(0, -2),
                                              Timestamp::fromSeconds(0));
    EXPECT_EQ(Point(), world.ball().position());
    EXPECT_EQ(Vector(0, -2), world.ball().velocity());
}

TEST(TestUtilsTest, has_all_valid_refbox_game_states)
{
    std::vector game_states = ::Test::TestUtil::getAllRefboxGameStates();
    // only way to test this getAllRefboxGameStates() without a literal copy-paste
    // of the implementation
    // note that this array does not contain RefboxGameState::REFBOX_GAME_STATE_COUNT,
    // this is intentional
    std::vector<RefboxGameState> expected_game_states = {
        RefboxGameState::HALT,
        RefboxGameState::STOP,
        RefboxGameState::NORMAL_START,
        RefboxGameState::FORCE_START,
        RefboxGameState::PREPARE_KICKOFF_US,
        RefboxGameState::PREPARE_KICKOFF_THEM,
        RefboxGameState::PREPARE_PENALTY_US,
        RefboxGameState::PREPARE_PENALTY_THEM,
        RefboxGameState::DIRECT_FREE_US,
        RefboxGameState::DIRECT_FREE_THEM,
        RefboxGameState::INDIRECT_FREE_US,
        RefboxGameState::INDIRECT_FREE_THEM,
        RefboxGameState::TIMEOUT_US,
        RefboxGameState::TIMEOUT_THEM,
        RefboxGameState::GOAL_US,
        RefboxGameState::GOAL_THEM,
        RefboxGameState::BALL_PLACEMENT_US,
        RefboxGameState::BALL_PLACEMENT_THEM};

    for (RefboxGameState state : expected_game_states)
    {
        EXPECT_TRUE(std::find(game_states.begin(), game_states.end(), state) !=
                    game_states.end());
    }
}

TEST(TestUtilsTest, polygon_check_if_equal_within_tolerance)
{
    Polygon poly1({Point(2.323, 2.113), Point(4.567, 1.069), Point(9.245, 1.227)});
    Polygon poly2({Point(2.324, 2.114), Point(4.568, 1.07), Point(9.246, 1.228)});
    Polygon poly3({Point(2.325, 2.115), Point(4.569, 1.071), Point(9.247, 1.229)});
    Polygon poly4(
        {Point(2.325, 2.115), Point(4.569, 1.071), Point(9.247, 1.229), Point(5, 5)});
    EXPECT_TRUE(
        ::Test::TestUtil::equalWithinTolerance(poly1, poly2, 2 * METERS_PER_MILLIMETER));
    EXPECT_FALSE(::Test::TestUtil::equalWithinTolerance(poly1, poly3));
    EXPECT_FALSE(::Test::TestUtil::equalWithinTolerance(poly1, poly4));
}

TEST(TestUtilsTest, point_check_if_equal_within_tolerance)
{
    Point point1(8.423, 4.913);
    Point point2(8.4232391, 4.9139881);
    Point point3(8.424, 4.914);
    Point point4(8.425, 4.915);
    Point point5(5.393, 1.113);
    Point point6(5.394, 1.114);
    Point point7(9.245, 1.227);
    Point point8(9.246, 1.227);

    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(point1, point2,
                                                       2 * METERS_PER_MILLIMETER));
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(point1, point3,
                                                       2 * METERS_PER_MILLIMETER));
    EXPECT_FALSE(::Test::TestUtil::equalWithinTolerance(point1, point4));
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(point5, point6,
                                                       2 * METERS_PER_MILLIMETER));
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(point6, point5,
                                                       2 * METERS_PER_MILLIMETER));
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(point7, point8));
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(point8, point7));
}

TEST(TestUtilsTest, circle_check_if_equal_within_tolerance)
{
    Circle circle1(Point(5.393, 1.113), 6.567);
    Circle circle2(Point(5.394, 1.114), 6.568);
    Circle circle3(Point(5.395, 1.115), 6.569);
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(circle1, circle2,
                                                       2 * METERS_PER_MILLIMETER));
    EXPECT_FALSE(::Test::TestUtil::equalWithinTolerance(circle1, circle2));
    EXPECT_FALSE(::Test::TestUtil::equalWithinTolerance(circle1, circle3));
}
