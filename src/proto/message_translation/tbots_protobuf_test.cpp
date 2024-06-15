#include "proto/message_translation/tbots_protobuf.h"

#include <gtest/gtest.h>
#include <string.h>

#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

class TbotsProtobufTest : public ::testing::Test
{
   public:
    static void assertPointMessageEqual(const Point& point,
                                        const TbotsProto::Point& point_msg)
    {
        EXPECT_EQ(point_msg.x_meters(), point.x());
        EXPECT_EQ(point_msg.y_meters(), point.y());
    }

    static void assertAngleMessageEqual(const Angle& angle,
                                        const TbotsProto::Angle& angle_msg)
    {
        EXPECT_EQ(angle_msg.radians(), angle.toRadians());
    }

    static void assertAngularVelocityMessageEqual(
        const AngularVelocity& angular_velocity,
        const TbotsProto::AngularVelocity& angular_velocity_msg)
    {
        EXPECT_EQ(angular_velocity_msg.radians_per_second(),
                  angular_velocity.toRadians());
    }

    static void assertVectorMessageEqual(const Vector& vector,
                                         const TbotsProto::Vector& vector_msg)
    {
        EXPECT_EQ(vector_msg.x_component_meters(), vector.x());
        EXPECT_EQ(vector_msg.y_component_meters(), vector.y());
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

    static void assertTrajectoryPathsAreSame(const TrajectoryPath& traj_path_1,
                                             const TrajectoryPath& traj_path_2)
    {
        auto traj_path_nodes_1 = traj_path_1.getTrajectoryPathNodes();
        auto traj_path_nodes_2 = traj_path_2.getTrajectoryPathNodes();
        ASSERT_EQ(traj_path_nodes_1.size(), traj_path_nodes_2.size());
        ASSERT_FLOAT_EQ(traj_path_nodes_1[0].getTrajectoryEndTime(),
                        traj_path_nodes_2[0].getTrajectoryEndTime());

        for (int i = 0; i < traj_path_nodes_1.size(); i++)
        {
            EXPECT_EQ(traj_path_nodes_1[i].getTrajectory()->getPosition(0.0),
                      traj_path_nodes_2[i].getTrajectory()->getPosition(0.0))
                << " Position at index " << i << " is not equal";
        }

        for (int i = 0; i < traj_path_nodes_1.size(); i++)
        {
            EXPECT_EQ(traj_path_nodes_1[i].getTrajectory()->getVelocity(0.0),
                      traj_path_nodes_2[i].getTrajectory()->getVelocity(0.0))
                << " Velocity at index " << i << " is not equal";
        }

        for (int i = 0; i < traj_path_nodes_1.size(); i++)
        {
            EXPECT_EQ(traj_path_nodes_1[i].getTrajectory()->getAcceleration(0.0),
                      traj_path_nodes_2[i].getTrajectory()->getAcceleration(0.0))
                << " Acceleration at index " << i << " is not equal";
        }

        for (int i = 0; i < traj_path_nodes_1.size(); i++)
        {
            EXPECT_EQ(traj_path_nodes_1[i].getTrajectory()->getDestination(),
                      traj_path_nodes_2[i].getTrajectory()->getDestination())
                << " Destination at index " << i << " is not equal";
        }
    }
};

TEST(TbotsProtobufTest, timestamp_msg_test)
{
    auto timestamp_msg = createCurrentTimestampProto();
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
    auto robot_state_msg = createRobotStateProto(robot);

    EXPECT_EQ(robot.currentState(), createRobotState(*robot_state_msg));
    TbotsProtobufTest::assertRobotStateMessageFromRobot(robot, *robot_state_msg);
}

TEST(TbotsProtobufTest, ball_state_msg_test)
{
    auto position = Point(4.20, 4.20);
    auto velocity = Vector(4.20, 4.20);

    Ball ball(position, velocity, Timestamp::fromSeconds(0));
    auto ball_state_msg = createBallStateProto(ball);

    TbotsProtobufTest::assertBallStateMessageFromBall(ball, *ball_state_msg);
}

class TrajectoryParamConversionTest
    : public ::testing::TestWithParam<std::tuple<std::vector<Point>, std::vector<double>>>
{
};

TEST_P(TrajectoryParamConversionTest, trajectory_params_msg_test)
{
    // Generate a trajectory, and then generate a TbotsProto::TrajectoryPathParams2D
    // with the same parameters as the trajectory, finally, generate a second trajectory
    // from the parameters and make sure the two trajectories are equal.
    RobotConstants robot_constants = create2021RobotConstants();
    Point start_position(0.0, 0.0);
    Point destination(0.0, 0.0);

    std::vector<Point> sub_destinations                  = std::get<0>(GetParam());
    std::vector<double> sub_destination_connection_times = std::get<1>(GetParam());
    ASSERT_EQ(sub_destinations.size(), sub_destination_connection_times.size());

    Vector initial_velocity(-1.0, -1.0);
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode =
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT;
    double max_speed = convertMaxAllowedSpeedModeToMaxAllowedSpeed(max_allowed_speed_mode,
                                                                   robot_constants);
    KinematicConstraints constraints(max_speed,
                                     robot_constants.robot_max_acceleration_m_per_s_2,
                                     robot_constants.robot_max_deceleration_m_per_s_2);

    Point initial_destination = destination;
    if (!sub_destinations.empty())
    {
        initial_destination = sub_destinations[0];
    }

    auto trajectory = std::make_shared<BangBangTrajectory2D>(
        start_position, initial_destination, initial_velocity, constraints);
    TrajectoryPath trajectory_path(trajectory, BangBangTrajectory2D::generator);

    for (int i = 1; i < sub_destinations.size(); i++)
    {
        trajectory_path.append(sub_destination_connection_times[i - 1],
                               sub_destinations[i], constraints);
    }

    if (!sub_destinations.empty())
    {
        trajectory_path.append(
            sub_destination_connection_times[sub_destination_connection_times.size() - 1],
            destination, constraints);
    }

    TbotsProto::TrajectoryPathParams2D params;
    *(params.mutable_start_position())   = *createPointProto(start_position);
    *(params.mutable_destination())      = *createPointProto(destination);
    *(params.mutable_initial_velocity()) = *createVectorProto(initial_velocity);
    params.set_max_speed_mode(max_allowed_speed_mode);

    for (int i = 0; i < sub_destinations.size(); ++i)
    {
        TbotsProto::TrajectoryPathParams2D::SubDestination sub_destination_proto;
        *(sub_destination_proto.mutable_sub_destination()) =
            *createPointProto(sub_destinations[i]);
        sub_destination_proto.set_connection_time_s(sub_destination_connection_times[i]);
        *(params.add_sub_destinations()) = sub_destination_proto;
    }

    auto converted_trajectory_path_opt =
        createTrajectoryPathFromParams(params, initial_velocity, robot_constants);
    ASSERT_TRUE(converted_trajectory_path_opt.has_value());

    TrajectoryPath converted_trajectory_path = converted_trajectory_path_opt.value();
    TbotsProtobufTest::assertTrajectoryPathsAreSame(trajectory_path,
                                                    converted_trajectory_path);
}

INSTANTIATE_TEST_CASE_P(
    TrajectoryParamConversionTests, TrajectoryParamConversionTest,
    ::testing::Values(
        // No sub destinations
        std::make_tuple<std::vector<Point>, std::vector<double>>({}, {}),
        // One sub destination
        std::make_tuple<std::vector<Point>, std::vector<double>>({Point(1, 1)}, {0.4}),
        // Multiple sub destination
        std::make_tuple<std::vector<Point>, std::vector<double>>(
            {Point(0, 2), Point(2, 2), Point(2, 0)}, {0.5, 1.0, 1.5})));
