#include "proto/message_translation/tbots_protobuf.h"

#include <gtest/gtest.h>
#include <string.h>

#include "software/test_util/test_util.h"

class TbotsProtobufTest : public ::testing::Test
{
   public:
    static void assertPointMessageEqual(const Point& point,
                                        const TbotsProto::Point& point_msg)
    {
        EXPECT_NEAR(point_msg.x_meters(), point.x(), 1e-6);
        EXPECT_NEAR(point_msg.y_meters(), point.y(), 1e-6);
    }

    static void assertAngleMessageEqual(const Angle& angle,
                                        const TbotsProto::Angle& angle_msg)
    {
        EXPECT_NEAR(angle_msg.radians(), angle.toRadians(), 1e-6);
    }

    static void assertAngularVelocityMessageEqual(
        const AngularVelocity& angular_velocity,
        const TbotsProto::AngularVelocity& angular_velocity_msg)
    {
        EXPECT_NEAR(angular_velocity_msg.radians_per_second(),
                    angular_velocity.toRadians(), 1e-6);
    }

    static void assertVectorMessageEqual(const Vector& vector,
                                         const TbotsProto::Vector& vector_msg)
    {
        EXPECT_NEAR(vector_msg.x_component_meters(), vector.x(), 1e-6);
        EXPECT_NEAR(vector_msg.y_component_meters(), vector.y(), 1e-6);
    }

    static void assertBallStateMessageFromBall(
        const Ball& ball, const TbotsProto::BallState& ball_state_msg)
    {
        assertPointMessageEqual(ball.position(), ball_state_msg.global_position());
        assertVectorMessageEqual(ball.velocity(), ball_state_msg.global_velocity());
    }

    static void assertRobotStateMessageFromRobot(
        const Robot& robot, const TbotsProto::RobotState& robot_state_msg)
    {
        assertPointMessageEqual(robot.position(), robot_state_msg.global_position());
        assertAngleMessageEqual(robot.orientation(),
                                robot_state_msg.global_orientation());
        assertVectorMessageEqual(robot.velocity(), robot_state_msg.global_velocity());
        assertAngularVelocityMessageEqual(robot.angularVelocity(),
                                          robot_state_msg.global_angular_velocity());
    }

    static void assertSaneTimestamp(const TbotsProto::Timestamp& timestamp)
    {
        // time will only move forward
        // we make sure the number that the timestamp is from the past
        const auto clock_time = std::chrono::system_clock::now();
        double time_in_seconds =
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                    clock_time.time_since_epoch())
                                    .count()) /
            MICROSECONDS_PER_SECOND;
        ASSERT_TRUE(timestamp.epoch_timestamp_seconds() <= time_in_seconds);
    }
};

TEST(TbotsProtobufTest, timestamp_msg_test)
{
    auto timestamp_msg = createCurrentTimestamp();
    TbotsProtobufTest::assertSaneTimestamp(*timestamp_msg);
}

TEST(TbotsProtobufTest, robot_state_msg_test)
{
    auto position         = Point(4.20, 4.20);
    auto velocity         = Vector(4.20, 4.20);
    auto orientation      = Angle::fromRadians(4.20);
    auto angular_velocity = Angle::fromRadians(4.20);

    Robot robot(0, position, velocity, orientation, angular_velocity,
                Timestamp::fromSeconds(0));
    auto robot_state_msg = createRobotState(robot);

    TbotsProtobufTest::assertRobotStateMessageFromRobot(robot, *robot_state_msg);
}

TEST(TbotsProtobufTest, ball_state_msg_test)
{
    auto position = Point(4.20, 4.20);
    auto velocity = Vector(4.20, 4.20);

    Ball ball(position, velocity, Timestamp::fromSeconds(0));
    auto ball_state_msg = createBallState(ball);

    TbotsProtobufTest::assertBallStateMessageFromBall(ball, *ball_state_msg);
}

TEST(TbotsProtobufTest, vision_msg_test)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setFriendlyRobotPositions(
        world,
        {Point(4.20, 4.20), Point(4.20, 4.20), Point(4.20, 4.20), Point(4.20, 4.20)},
        Timestamp::fromSeconds(0));

    auto vision_msg = createVision(world);

    TbotsProtobufTest::assertBallStateMessageFromBall(world.ball(),
                                                      vision_msg->ball_state());
    TbotsProtobufTest::assertSaneTimestamp(vision_msg->time_sent());

    auto friendly_robots   = world.friendlyTeam().getAllRobots();
    auto& robot_states_map = *vision_msg->mutable_robot_states();

    std::for_each(friendly_robots.begin(), friendly_robots.end(),
                  [&](const Robot& robot) {
                      ASSERT_TRUE(robot_states_map.count(robot.id()));
                      TbotsProtobufTest::assertRobotStateMessageFromRobot(
                          robot, robot_states_map[robot.id()]);
                  });
}
