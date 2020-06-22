#include "software/world/world.h"

#include <gtest/gtest.h>

#include "software/parameter/dynamic_parameters.h"
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

// Test that most recent timestamp from member objects works
TEST_F(WorldTest, get_most_recent_timestamp_from_members)
{
    EXPECT_EQ(world.getMostRecentTimestamp(), current_time);
}

// Test that the timestamp history is accurate
TEST_F(WorldTest, get_timestamp_history)
{
    Timestamp timestamp_1 = current_time + Duration::fromSeconds(0);
    world.updateTimestamp(timestamp_1);

    Timestamp timestamp_2 = current_time + Duration::fromSeconds(1);
    world.updateTimestamp(timestamp_2);

    Timestamp timestamp_3 = current_time + Duration::fromSeconds(2);
    world.updateTimestamp(timestamp_3);

    EXPECT_EQ(world.getTimestampHistory()[3], current_time);
    EXPECT_EQ(world.getTimestampHistory()[2], timestamp_1);
    EXPECT_EQ(world.getTimestampHistory()[1], timestamp_2);
    EXPECT_EQ(world.getTimestampHistory()[0], timestamp_3);
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
