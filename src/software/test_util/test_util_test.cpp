#include "software/test_util/test_util.h"

#include <gtest/gtest.h>

#include <algorithm>

/*
 * Unit tests for the unit test utilities
 */
TEST(TestUtilsTest, create_division_b_field)
{
    Field field = ::TestUtil::createSSLDivBField();

    // Check that the field has the correct dimensions for a
    // SSL Division B field according to the rules
    EXPECT_DOUBLE_EQ(9.0, field.xLength());
    EXPECT_DOUBLE_EQ(6.0, field.yLength());
    EXPECT_DOUBLE_EQ(9.6, field.totalXLength());
    EXPECT_DOUBLE_EQ(6.6, field.totalYLength());

    EXPECT_DOUBLE_EQ(1.0, field.goalYLength());
    EXPECT_DOUBLE_EQ(0.18, field.goalXLength());

    EXPECT_DOUBLE_EQ(0.5, field.centerCircleRadius());
    EXPECT_EQ(Point(0, 0), field.centerPoint());
    EXPECT_EQ(Circle(Point(0, 0), 0.5), field.centerCircle());

    EXPECT_EQ(Segment(Point(0, 3), Point(0, -3)), field.halfwayLine());

    EXPECT_DOUBLE_EQ(2.0, field.defenseAreaYLength());
    EXPECT_DOUBLE_EQ(1.0, field.defenseAreaXLength());
    EXPECT_EQ(Rectangle(Point(-4.5, 1.0), Point(-3.5, -1.0)),
              field.friendlyDefenseArea());
    EXPECT_EQ(Rectangle(Point(4.5, 1.0), Point(3.5, -1.0)), field.enemyDefenseArea());

    EXPECT_EQ(Rectangle(Point(-4.5, -3.0), Point(0, 3.0)), field.friendlyHalf());
    EXPECT_EQ(Rectangle(Point(-4.5, 0), Point(0, 3.0)),
              field.friendlyPositiveYQuadrant());
    EXPECT_EQ(Rectangle(Point(-4.5, 0), Point(0, -3.0)),
              field.friendlyNegativeYQuadrant());
    EXPECT_EQ(Rectangle(Point(0, -3.0), Point(4.5, 3.0)), field.enemyHalf());
    EXPECT_EQ(Rectangle(Point(0, 0), Point(4.5, 3.0)), field.enemyPositiveYQuadrant());
    EXPECT_EQ(Rectangle(Point(0, 0), Point(4.5, -3.0)), field.enemyNegativeYQuadrant());

    EXPECT_EQ(Rectangle(Point(-4.5, -3.0), Point(4.5, 3.0)), field.fieldLines());
    EXPECT_EQ(Rectangle(Point(-4.8, -3.3), Point(4.8, 3.3)), field.fieldBoundary());

    EXPECT_EQ(Point(-4.5, 0.0), field.friendlyGoalCenter());
    EXPECT_EQ(Point(4.5, 0.0), field.enemyGoalCenter());
    EXPECT_EQ(Rectangle(Point(-4.68, -0.5), Point(-4.5, 0.5)).getPoints(),
              field.friendlyGoal().getPoints());
    EXPECT_EQ(Rectangle(Point(4.68, -0.5), Point(4.5, 0.5)).getPoints(),
              field.enemyGoal().getPoints());

    EXPECT_EQ(Point(-3.5, 0.0), field.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 3.0), field.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -3.0), field.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 3.0), field.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -3.0), field.enemyCornerNeg());

    EXPECT_EQ(Point(-4.5, 0.5), field.friendlyGoalpostPos());
    EXPECT_EQ(Point(-4.5, -0.5), field.friendlyGoalpostNeg());
    EXPECT_EQ(Point(4.5, 0.5), field.enemyGoalpostPos());
    EXPECT_EQ(Point(4.5, -0.5), field.enemyGoalpostNeg());

    EXPECT_DOUBLE_EQ(0.3, field.boundaryMargin());
}

TEST(TestUtilsTest, create_division_a_field)
{
    Field field = ::TestUtil::createSSLDivAField();

    // Check that the field has the correct dimensions for a
    // SSL Division A field according to the rules
    EXPECT_DOUBLE_EQ(12.0, field.xLength());
    EXPECT_DOUBLE_EQ(9.0, field.yLength());
    EXPECT_DOUBLE_EQ(12.6, field.totalXLength());
    EXPECT_DOUBLE_EQ(9.6, field.totalYLength());

    EXPECT_DOUBLE_EQ(1.8, field.goalYLength());
    EXPECT_DOUBLE_EQ(0.18, field.goalXLength());

    EXPECT_DOUBLE_EQ(0.5, field.centerCircleRadius());
    EXPECT_EQ(Point(0, 0), field.centerPoint());
    EXPECT_EQ(Circle(Point(0, 0), 0.5), field.centerCircle());

    EXPECT_EQ(Segment(Point(0, 4.5), Point(0, -4.5)), field.halfwayLine());

    EXPECT_DOUBLE_EQ(3.6, field.defenseAreaYLength());
    EXPECT_DOUBLE_EQ(1.8, field.defenseAreaXLength());
    EXPECT_EQ(Rectangle(Point(-6, 1.8), Point(-4.2, -1.8)), field.friendlyDefenseArea());
    EXPECT_EQ(Rectangle(Point(6, 1.8), Point(4.2, -1.8)), field.enemyDefenseArea());

    EXPECT_EQ(Rectangle(Point(-6, -4.5), Point(0, 4.5)), field.friendlyHalf());
    EXPECT_EQ(Rectangle(Point(-6, 0), Point(0, 4.5)), field.friendlyPositiveYQuadrant());
    EXPECT_EQ(Rectangle(Point(-6, 0), Point(0, -4.5)), field.friendlyNegativeYQuadrant());
    EXPECT_EQ(Rectangle(Point(0, -4.5), Point(6, 4.5)), field.enemyHalf());
    EXPECT_EQ(Rectangle(Point(0, 0), Point(6, 4.5)), field.enemyPositiveYQuadrant());
    EXPECT_EQ(Rectangle(Point(0, 0), Point(6, -4.5)), field.enemyNegativeYQuadrant());

    EXPECT_EQ(Rectangle(Point(-6, -4.5), Point(6, 4.5)), field.fieldLines());
    EXPECT_EQ(Rectangle(Point(-6.3, -4.8), Point(6.3, 4.8)), field.fieldBoundary());

    EXPECT_EQ(Point(-6, 0.0), field.friendlyGoalCenter());
    EXPECT_EQ(Point(6, 0.0), field.enemyGoalCenter());
    EXPECT_EQ(Rectangle(Point(-6.18, -0.9), Point(-6, 0.9)).getPoints(),
              field.friendlyGoal().getPoints());
    EXPECT_EQ(Rectangle(Point(6.18, -0.9), Point(6, 0.9)).getPoints(),
              field.enemyGoal().getPoints());

    EXPECT_EQ(Point(-4.2, 0.0), field.penaltyFriendly());
    EXPECT_EQ(Point(4.2, 0.0), field.penaltyEnemy());

    EXPECT_EQ(Point(-6, 4.5), field.friendlyCornerPos());
    EXPECT_EQ(Point(-6, -4.5), field.friendlyCornerNeg());
    EXPECT_EQ(Point(6, 4.5), field.enemyCornerPos());
    EXPECT_EQ(Point(6, -4.5), field.enemyCornerNeg());

    EXPECT_EQ(Point(-6, 0.9), field.friendlyGoalpostPos());
    EXPECT_EQ(Point(-6, -0.9), field.friendlyGoalpostNeg());
    EXPECT_EQ(Point(6, 0.9), field.enemyGoalpostPos());
    EXPECT_EQ(Point(6, -0.9), field.enemyGoalpostNeg());

    EXPECT_DOUBLE_EQ(0.3, field.boundaryMargin());
}

TEST(TestUtilsTest, create_testing_world)
{
    World world = ::TestUtil::createBlankTestingWorld();

    EXPECT_EQ(::TestUtil::createSSLDivBField(), world.field());
    EXPECT_EQ(Team(Duration::fromMilliseconds(1000)), world.friendlyTeam());
    EXPECT_EQ(Team(Duration::fromMilliseconds(1000)), world.enemyTeam());
    EXPECT_EQ(Ball(Point(), Vector(), Timestamp::fromSeconds(0)), world.ball());
}

TEST(TestUtilsTest, set_friendly_robot_positions_in_world_with_positive_number_of_robots)
{
    World world = ::TestUtil::createBlankTestingWorld();

    world = ::TestUtil::setFriendlyRobotPositions(
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
    World world = ::TestUtil::createBlankTestingWorld();

    world = ::TestUtil::setEnemyRobotPositions(
        world, {world.field().enemyGoalCenter(), world.field().friendlyCornerPos()},
        Timestamp::fromSeconds(0));

    EXPECT_EQ(2, world.enemyTeam().numRobots());
    EXPECT_EQ(world.field().enemyGoalCenter(),
              (*world.enemyTeam().getRobotById(0)).position());
    EXPECT_EQ(world.field().friendlyCornerPos(),
              (*world.enemyTeam().getRobotById(1)).position());
    EXPECT_EQ(Vector(), (*world.enemyTeam().getRobotById(0)).velocity());
    EXPECT_EQ(Angle::zero(), (*world.enemyTeam().getRobotById(0)).orientation());
}

TEST(TestUtilsTest, set_friendly_robot_positions_in_world_with_zero_robots)
{
    World world = ::TestUtil::createBlankTestingWorld();

    world = ::TestUtil::setFriendlyRobotPositions(world, {}, Timestamp::fromSeconds(0));

    EXPECT_EQ(0, world.friendlyTeam().numRobots());
}

TEST(TestUtilsTest, set_enemy_robot_positions_in_world_with_zero_robots)
{
    World world = ::TestUtil::createBlankTestingWorld();

    world = ::TestUtil::setEnemyRobotPositions(world, {}, Timestamp::fromSeconds(0));

    EXPECT_EQ(0, world.enemyTeam().numRobots());
}

TEST(TestUtilsTest, set_ball_position_in_world)
{
    World world = ::TestUtil::createBlankTestingWorld();

    world =
        ::TestUtil::setBallPosition(world, Point(-0.2, 3.11), Timestamp::fromSeconds(0));
    EXPECT_EQ(Point(-0.2, 3.11), world.ball().position());
    EXPECT_EQ(Vector(), world.ball().velocity());
}

TEST(TestUtilsTest, set_ball_velocity_in_world)
{
    World world = ::TestUtil::createBlankTestingWorld();

    world = ::TestUtil::setBallVelocity(world, Vector(0, -2), Timestamp::fromSeconds(0));
    EXPECT_EQ(Point(), world.ball().position());
    EXPECT_EQ(Vector(0, -2), world.ball().velocity());
}

TEST(TestUtilsTest, has_all_valid_refbox_game_states)
{
    std::vector game_states = ::TestUtil::getAllRefboxGameStates();
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
        ::TestUtil::equalWithinTolerance(poly1, poly2, 2 * METERS_PER_MILLIMETER));
    EXPECT_FALSE(::TestUtil::equalWithinTolerance(poly1, poly3));
    EXPECT_FALSE(::TestUtil::equalWithinTolerance(poly1, poly4));
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

    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(point1, point2, 2 * METERS_PER_MILLIMETER));
    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(point1, point3, 2 * METERS_PER_MILLIMETER));
    EXPECT_FALSE(::TestUtil::equalWithinTolerance(point1, point4));
    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(point5, point6, 2 * METERS_PER_MILLIMETER));
    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(point6, point5, 2 * METERS_PER_MILLIMETER));
    EXPECT_TRUE(::TestUtil::equalWithinTolerance(point7, point8));
    EXPECT_TRUE(::TestUtil::equalWithinTolerance(point8, point7));
}

TEST(TestUtilsTest, circle_check_if_equal_within_tolerance)
{
    Circle circle1(Point(5.393, 1.113), 6.567);
    Circle circle2(Point(5.394, 1.114), 6.568);
    Circle circle3(Point(5.395, 1.115), 6.569);
    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(circle1, circle2, 2 * METERS_PER_MILLIMETER));
    EXPECT_FALSE(::TestUtil::equalWithinTolerance(circle1, circle2));
    EXPECT_FALSE(::TestUtil::equalWithinTolerance(circle1, circle3));
}

TEST(TestUtilsTest, vector_check_if_equal_within_tolerance)
{
    Vector vector1(8.423, 4.913);
    Vector vector2(8.4232391, 4.9139881);
    Vector vector3(8.424, 4.914);
    Vector vector4(8.425, 4.915);
    Vector vector5(5.393, 1.113);
    Vector vector6(5.394, 1.114);
    Vector vector7(9.245, 1.227);
    Vector vector8(9.246, 1.227);

    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(vector1, vector2, 2 * METERS_PER_MILLIMETER));
    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(vector1, vector3, 2 * METERS_PER_MILLIMETER));
    EXPECT_FALSE(::TestUtil::equalWithinTolerance(vector1, vector4));
    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(vector5, vector6, 2 * METERS_PER_MILLIMETER));
    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(vector6, vector5, 2 * METERS_PER_MILLIMETER));
    EXPECT_TRUE(::TestUtil::equalWithinTolerance(vector7, vector8));
    EXPECT_TRUE(::TestUtil::equalWithinTolerance(vector8, vector7));
}

TEST(TestUtilsTest, angle_check_if_equal_within_tolerance)
{
    Angle angle1 = Angle::fromDegrees(5);
    Angle angle2 = Angle::fromDegrees(5.5);
    Angle angle3 = Angle::fromDegrees(189);
    Angle angle4 = Angle::fromDegrees(190);
    Angle angle5 = Angle::fromDegrees(-10);

    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(angle1, angle2, Angle::fromDegrees(0.5)));
    EXPECT_FALSE(
        ::TestUtil::equalWithinTolerance(angle1, angle2, Angle::fromDegrees(0.49)));
    EXPECT_TRUE(::TestUtil::equalWithinTolerance(angle3, angle4, Angle::fromDegrees(1)));
    EXPECT_FALSE(
        ::TestUtil::equalWithinTolerance(angle3, angle4, Angle::fromDegrees(0.99)));
    EXPECT_TRUE(::TestUtil::equalWithinTolerance(angle1, angle5, Angle::fromDegrees(15)));
    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(angle4, angle5, Angle::fromDegrees(180)));
}

TEST(TestUtilsTest, robot_state_check_if_equal_within_tolerance)
{
    RobotState state1(Point(1.01, 2.58), Vector(0.0, -2.06), Angle::fromDegrees(4),
                      AngularVelocity::fromDegrees(67.4));
    RobotState state2(Point(1.01, 2.59), Vector(0.005, -2.05), Angle::fromDegrees(5),
                      AngularVelocity::fromDegrees(67.6));
    RobotState state3(Point(1.11, 2.59), Vector(0.0, -2.06), Angle::fromDegrees(4),
                      AngularVelocity::fromDegrees(67.4));
    RobotState state4(Point(1.01, 2.58), Vector(0.4, -2.58), Angle::fromDegrees(4),
                      AngularVelocity::fromDegrees(67.4));
    RobotState state5(Point(1.01, 2.58), Vector(0.0, -2.06), Angle::fromDegrees(6.4),
                      AngularVelocity::fromDegrees(67.4));
    RobotState state6(Point(1.01, 2.58), Vector(0.0, -2.06), Angle::fromDegrees(4),
                      AngularVelocity::fromDegrees(70.02));

    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(state1, state2, 0.02, Angle::fromDegrees(1)));
    EXPECT_FALSE(
        ::TestUtil::equalWithinTolerance(state1, state3, 0.1, Angle::fromDegrees(1)));
    EXPECT_FALSE(
        ::TestUtil::equalWithinTolerance(state1, state4, 0.5, Angle::fromDegrees(1)));
    EXPECT_FALSE(
        ::TestUtil::equalWithinTolerance(state1, state5, 1e-3, Angle::fromDegrees(2)));
    EXPECT_FALSE(
        ::TestUtil::equalWithinTolerance(state1, state6, 1e-3, Angle::fromDegrees(2.5)));
}

TEST(TestUtilsTest, ball_state_check_if_equal_within_tolerance)
{
    BallState state1(Point(1.01, 2.58), Vector(0.0, -2.06));
    BallState state2(Point(1.01, 2.59), Vector(0.005, -2.05));
    BallState state3(Point(1.11, 2.59), Vector(0.0, -2.06));
    BallState state4(Point(1.01, 2.58), Vector(0.4, -2.58));

    EXPECT_TRUE(::TestUtil::equalWithinTolerance(state1, state2, 0.02));
    EXPECT_FALSE(::TestUtil::equalWithinTolerance(state1, state3, 0.1));
    EXPECT_FALSE(::TestUtil::equalWithinTolerance(state1, state4, 0.5));
}

TEST(TestUtilsTest, test_seconds_since)
{
    const auto start_time = std::chrono::system_clock::now();
    EXPECT_TRUE(::TestUtil::secondsSince(start_time) > 0);
}

TEST(TestUtilsTest, test_milliseconds_since)
{
    const auto start_time = std::chrono::system_clock::now();
    EXPECT_TRUE(::TestUtil::millisecondsSince(start_time) > 0);
}
