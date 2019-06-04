#include "test/test_util/test_util.h"

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
    EXPECT_DOUBLE_EQ(9.6, field.totalLength());
    EXPECT_DOUBLE_EQ(6.6, field.totalWidth());
    EXPECT_DOUBLE_EQ(0.3, field.boundaryWidth());

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

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
