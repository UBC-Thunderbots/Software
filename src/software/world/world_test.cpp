#include "software/world/world.h"

#include <gtest/gtest.h>
#include <include/gmock/gmock-matchers.h>

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/parameters.pb.h"
#include "software/test_util/test_util.h"

class WorldTest : public ::testing::Test
{
   protected:
    WorldTest()
        : current_time(Timestamp::fromSeconds(123)),
          field(Field::createSSLDivisionBField()),
          ball(Point(1, 2), Vector(-0.3, 0), current_time),
          friendly_team(Duration::fromMilliseconds(1000)),
          enemy_team(Duration::fromMilliseconds(1000)),
          world(field, ball, friendly_team, enemy_team)
    {
    }

    void SetUp() override
    {
        // An arbitrary fixed point in time
        // We use this fixed point in time to make the tests deterministic.
        Robot friendly_robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                                       AngularVelocity::threeQuarter(), current_time);

        Robot friendly_robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                                       AngularVelocity::zero(), current_time);

        friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});
        friendly_team.assignGoalie(1);

        Robot enemy_robot_0 = Robot(0, Point(0.5, -2.5), Vector(), Angle::fromRadians(1),
                                    AngularVelocity::fromRadians(2), current_time);

        Robot enemy_robot_1 = Robot(1, Point(), Vector(-0.5, 4), Angle::quarter(),
                                    AngularVelocity::half(), current_time);

        enemy_team.updateRobots({enemy_robot_0, enemy_robot_1});
        enemy_team.assignGoalie(0);

        // Construct the world with arguments
        world = World(field, ball, friendly_team, enemy_team);
    }

    Timestamp current_time;
    Field field;
    Ball ball;
    Team friendly_team;
    Team enemy_team;
    World world;
};

TEST_F(WorldTest, construction_with_parameters)
{
    // Check that objects used for construction are returned by the accessors
    EXPECT_EQ(field, world.field());
    EXPECT_EQ(ball, world.ball());
    EXPECT_EQ(friendly_team, world.friendlyTeam());
    EXPECT_EQ(enemy_team, world.enemyTeam());
}

TEST_F(WorldTest, construct_with_protobuf)
{
    auto world_proto = createWorld(world);
    World proto_converted_world(*world_proto);

    // Can not compare the two World objects since TbotsProto::Team does not store the
    // robot_expiry_buffer_duration
    EXPECT_EQ(world.field(), proto_converted_world.field());
    EXPECT_EQ(world.ball(), proto_converted_world.ball());
    EXPECT_EQ(world.gameState(), proto_converted_world.gameState());
    EXPECT_EQ(world.enemyTeam().getGoalieId(),
              proto_converted_world.enemyTeam().getGoalieId());
    EXPECT_THAT(
        world.friendlyTeam().getAllRobots(),
        ::testing::ContainerEq(proto_converted_world.friendlyTeam().getAllRobots()));
    EXPECT_THAT(world.enemyTeam().getAllRobots(),
                ::testing::ContainerEq(proto_converted_world.enemyTeam().getAllRobots()));
}

// Test that most recent timestamp from member objects works
TEST_F(WorldTest, get_most_recent_timestamp_from_members)
{
    EXPECT_EQ(world.getMostRecentTimestamp(), current_time);
}

TEST_F(WorldTest, equality_basic_tests)
{
    World world1 = world;
    World world2 = world;
    EXPECT_EQ(world1, world2);
    EXPECT_EQ(world2, world1);
    EXPECT_EQ(world1, world1);
}

TEST_F(WorldTest, equality_different_timestamp)
{
    World world1 = world;
    World world3 = world;
    world3.updateTimestamp(current_time + Duration::fromSeconds(100));
    EXPECT_EQ(world1, world3);
}

TEST_F(WorldTest, equality_different_ball)
{
    World world1       = world;
    Ball ball          = Ball(Point(1, 0), Vector(13, 0), Timestamp::fromSeconds(0));
    Team friendly_team = Team(Duration::fromMilliseconds(0));
    Team enemy_team    = Team(Duration::fromMilliseconds(0));
    World world2       = World(field, ball, friendly_team, enemy_team);
    EXPECT_NE(world1, world2);
}

TEST_F(WorldTest, equality_different_field)
{
    Field field1       = field;
    Field field2       = Field(2, 5, 3, 1, 3, 1, 2, 8);
    Ball ball          = Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    Team friendly_team = Team(Duration::fromMilliseconds(0));
    Team enemy_team    = Team(Duration::fromMilliseconds(0));
    World world1       = World(field1, ball, friendly_team, enemy_team);
    World world2       = World(field2, ball, friendly_team, enemy_team);
    EXPECT_NE(world1, world2);
}

TEST_F(WorldTest, equality_different_friendly_team)
{
    World world1       = world;
    Ball ball          = Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    Team friendly_team = Team(Duration::fromMilliseconds(1500));
    Team enemy_team    = Team(Duration::fromMilliseconds(0));
    World world2       = World(field, ball, friendly_team, enemy_team);
    EXPECT_NE(world1, world2);
}

TEST_F(WorldTest, equality_different_enemy_team)
{
    World world1       = world;
    Ball ball          = Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    Team friendly_team = Team(Duration::fromMilliseconds(0));
    Team enemy_team    = Team(Duration::fromMilliseconds(1300));
    World world2       = World(field, ball, friendly_team, enemy_team);
    EXPECT_NE(world1, world2);
}

TEST_F(WorldTest, update_referee_command)
{
    world.updateRefereeCommand(RefereeCommand::HALT);
    EXPECT_EQ(world.gameState().getRefereeCommand(), RefereeCommand::HALT);
    for (unsigned int i = 0; i < World::REFEREE_COMMAND_BUFFER_SIZE - 1; i++)
    {
        world.updateRefereeCommand(RefereeCommand::FORCE_START);
        EXPECT_NE(world.gameState().getRefereeCommand(), RefereeCommand::FORCE_START);
    }

    for (unsigned int i = 0; i < World::REFEREE_COMMAND_BUFFER_SIZE; i++)
    {
        world.updateRefereeCommand(RefereeCommand::FORCE_START);
        EXPECT_EQ(world.gameState().getRefereeCommand(), RefereeCommand::FORCE_START);
    }

    for (unsigned int i = 0; i < World::REFEREE_COMMAND_BUFFER_SIZE - 1; i++)
    {
        world.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_US, Point(0, 0));
        EXPECT_NE(world.gameState().getRefereeCommand(),
                  RefereeCommand::BALL_PLACEMENT_US);
    }

    for (unsigned int i = 0; i < World::REFEREE_COMMAND_BUFFER_SIZE; i++)
    {
        world.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_US, Point(0, 0));
        EXPECT_EQ(world.gameState().getRefereeCommand(),
                  RefereeCommand::BALL_PLACEMENT_US);
    }
}

TEST_F(WorldTest, update_referee_stage)
{
    world.updateRefereeStage(RefereeStage::NORMAL_FIRST_HALF_PRE);
    EXPECT_EQ(world.getRefereeStage(), RefereeStage::NORMAL_FIRST_HALF_PRE);
    for (unsigned int i = 0; i < World::REFEREE_COMMAND_BUFFER_SIZE - 1; i++)
    {
        world.updateRefereeStage(RefereeStage::NORMAL_FIRST_HALF);
        EXPECT_NE(world.getRefereeStage(), RefereeStage::NORMAL_FIRST_HALF);
    }

    for (unsigned int i = 0; i < World::REFEREE_COMMAND_BUFFER_SIZE; i++)
    {
        world.updateRefereeStage(RefereeStage::NORMAL_FIRST_HALF);
        EXPECT_EQ(world.getRefereeStage(), RefereeStage::NORMAL_FIRST_HALF);
    }

    for (unsigned int i = 0; i < World::REFEREE_COMMAND_BUFFER_SIZE - 1; i++)
    {
        world.updateRefereeStage(RefereeStage::NORMAL_SECOND_HALF_PRE);
        EXPECT_NE(world.getRefereeStage(), RefereeStage::NORMAL_SECOND_HALF_PRE);
    }

    for (unsigned int i = 0; i < World::REFEREE_COMMAND_BUFFER_SIZE; i++)
    {
        world.updateRefereeStage(RefereeStage::NORMAL_SECOND_HALF_PRE);
        EXPECT_EQ(world.getRefereeStage(), RefereeStage::NORMAL_SECOND_HALF_PRE);
    }
}

TEST_F(WorldTest, set_team_with_possession)
{
    world.setTeamWithPossession(TeamPossession::FRIENDLY_TEAM);
    EXPECT_EQ(world.getTeamWithPossession(), TeamPossession::FRIENDLY_TEAM);
    world.setTeamWithPossession(TeamPossession::ENEMY_TEAM);
    EXPECT_EQ(world.getTeamWithPossession(), TeamPossession::ENEMY_TEAM);
}
