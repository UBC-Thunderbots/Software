#include "software/test_util/test_util.h"

#include <gtest/gtest.h>

#include <algorithm>

/*
 * Unit tests for the unit test utilities
 */

TEST(TestUtilsTest, create_testing_world)
{
    World world = ::TestUtil::createBlankTestingWorld();

    EXPECT_EQ(Field::createSSLDivisionBField(), world.field());
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

TEST(TestUtilsTest, test_create_stationary_robot_states_with_id)
{
    std::vector<RobotStateWithId> expected = {
        RobotStateWithId{
            .id          = 0,
            .robot_state = RobotState(Point(-3, 2.5), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 1,
            .robot_state = RobotState(Point(-3, 1.5), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 2,
            .robot_state = RobotState(Point(-3, 0.5), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 3,
            .robot_state = RobotState(Point(-3, -0.5), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 4,
            .robot_state = RobotState(Point(-3, -1.5), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 5,
            .robot_state = RobotState(Point(4.6, -3.1), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
    };

    EXPECT_EQ(expected, TestUtil::createStationaryRobotStatesWithId(
                            {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5),
                             Point(-3, -0.5), Point(-3, -1.5), Point(4.6, -3.1)}));
}
