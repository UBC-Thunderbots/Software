#include "software/world/world.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"
#include "software/util/parameter/dynamic_parameters.h"

class WorldTest : public ::testing::Test
{
   protected:
    WorldTest()
        : current_time(Timestamp::fromSeconds(123)),
          field(0, 0, 0, 0, 0, 0, 0, current_time),
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
        field = ::Test::TestUtil::createSSLDivBField();

        Robot friendly_robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                                       AngularVelocity::threeQuarter(), current_time);

        Robot friendly_robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                                       AngularVelocity::zero(), current_time);

        friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});
        friendly_team.assignGoalie(1);

        Robot enemy_robot_0 = Robot(0, Point(0.5, -2.5), Vector(), Angle::ofRadians(1),
                                    AngularVelocity::ofRadians(2), current_time);

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

TEST_F(WorldTest, default_constructor)
{
    World world;
    // Check that objects used for construction are returned by the accessors
    EXPECT_EQ(Field(0, 0, 0, 0, 0, 0, 0, current_time), world.field());
    EXPECT_EQ(Ball(Point(), Vector(), Timestamp::fromSeconds(0)), world.ball());
    EXPECT_EQ(Team(Duration::fromMilliseconds(
                  Util::DynamicParameters->RobotExpiryBufferMilliseconds()->value())),
              world.friendlyTeam());
    EXPECT_EQ(Team(Duration::fromMilliseconds(
                  Util::DynamicParameters->RobotExpiryBufferMilliseconds()->value())),
              world.enemyTeam());
}

TEST_F(WorldTest, construction_with_parameters)
{
    // Check that objects used for construction are returned by the accessors
    EXPECT_EQ(field, world.field());
    EXPECT_EQ(ball, world.ball());
    EXPECT_EQ(friendly_team, world.friendlyTeam());
    EXPECT_EQ(enemy_team, world.enemyTeam());
}

// Test that getMostRecentTimestamp functions properly
TEST_F(WorldTest, get_most_recent_from_members)
{
    World world;

    EXPECT_EQ(world.getMostRecentTimestamp(), Timestamp::fromSeconds(0));
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
