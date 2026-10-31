#include "software/simulation/simulated_imu.h"

#include <gtest/gtest.h>

class SimulatedImuServiceTest : public ::testing::Test
{
   protected:
    static RobotState makeTrueState(const Vector& velocity, const Angle& orientation,
                                    const AngularVelocity& angular_velocity)
    {
        return RobotState(Point(0, 0), velocity, orientation, angular_velocity);
    }

    SimulatedImuService imu_service{{}, 0};
    TbotsProto::RobotStatus robot_status;
};

TEST_F(SimulatedImuServiceTest, reports_true_angular_velocity)
{
    imu_service.poll(
        makeTrueState(Vector(), Angle::zero(), AngularVelocity::fromRadians(0.7)),
        robot_status, Duration::fromSeconds(0.01));

    EXPECT_NEAR(robot_status.imu_status().angular_velocity().radians_per_second(), 0.7,
                1e-9);
}

TEST_F(SimulatedImuServiceTest, first_poll_has_no_accelerations)
{
    imu_service.poll(
        makeTrueState(Vector(1, 0), Angle::zero(), AngularVelocity::fromRadians(0.7)),
        robot_status, Duration::fromSeconds(0.01));

    EXPECT_TRUE(robot_status.imu_status().has_angular_velocity());
    EXPECT_FALSE(robot_status.imu_status().has_angular_acceleration());
    EXPECT_FALSE(robot_status.imu_status().has_linear_acceleration());
}

TEST_F(SimulatedImuServiceTest, angular_acceleration_is_change_in_angular_velocity)
{
    imu_service.poll(
        makeTrueState(Vector(), Angle::zero(), AngularVelocity::fromRadians(0.5)),
        robot_status, Duration::fromSeconds(0.1));
    imu_service.poll(
        makeTrueState(Vector(), Angle::zero(), AngularVelocity::fromRadians(1.0)),
        robot_status, Duration::fromSeconds(0.1));

    EXPECT_NEAR(
        robot_status.imu_status().angular_acceleration().radians_per_second_squared(),
        5.0, 1e-9);
}

TEST_F(SimulatedImuServiceTest, linear_acceleration_is_in_robot_frame)
{
    // The robot faces +y and speeds up along +x, which is to its right
    imu_service.poll(makeTrueState(Vector(0, 0), Angle::quarter(), AngularVelocity()),
                     robot_status, Duration::fromSeconds(0.5));
    imu_service.poll(makeTrueState(Vector(1, 0), Angle::quarter(), AngularVelocity()),
                     robot_status, Duration::fromSeconds(0.5));

    const auto& linear_acceleration = robot_status.imu_status().linear_acceleration();
    EXPECT_NEAR(linear_acceleration.x_component_meters(), 0.0, 1e-9);
    EXPECT_NEAR(linear_acceleration.y_component_meters(), -2.0, 1e-9);
}

TEST_F(SimulatedImuServiceTest, bias_offsets_readings_but_cancels_in_angular_acceleration)
{
    SimulatedImuService biased_imu_service(
        {.angular_velocity_bias_rad_per_s    = 0.1,
         .linear_acceleration_bias_m_per_s_2 = Vector(0.2, -0.3)},
        0);
    const RobotState true_state =
        makeTrueState(Vector(1, 0), Angle::zero(), AngularVelocity::fromRadians(0.5));

    biased_imu_service.poll(true_state, robot_status, Duration::fromSeconds(0.1));
    biased_imu_service.poll(true_state, robot_status, Duration::fromSeconds(0.1));

    const auto& imu_status = robot_status.imu_status();
    EXPECT_NEAR(imu_status.angular_velocity().radians_per_second(), 0.6, 1e-9);
    EXPECT_NEAR(imu_status.angular_acceleration().radians_per_second_squared(), 0.0,
                1e-9);
    EXPECT_NEAR(imu_status.linear_acceleration().x_component_meters(), 0.2, 1e-9);
    EXPECT_NEAR(imu_status.linear_acceleration().y_component_meters(), -0.3, 1e-9);
}

TEST_F(SimulatedImuServiceTest, same_seed_gives_same_noisy_readings)
{
    const SimulatedImuService::NoiseConfig noise_config{
        .angular_velocity_stddev_rad_per_s    = 0.05,
        .linear_acceleration_stddev_m_per_s_2 = 0.1};
    SimulatedImuService first_imu_service(noise_config, 7);
    SimulatedImuService second_imu_service(noise_config, 7);
    const RobotState true_state =
        makeTrueState(Vector(1, 0), Angle::zero(), AngularVelocity::fromRadians(0.5));

    TbotsProto::RobotStatus first_robot_status;
    TbotsProto::RobotStatus second_robot_status;
    for (int i = 0; i < 2; i++)
    {
        first_imu_service.poll(true_state, first_robot_status,
                               Duration::fromSeconds(0.1));
        second_imu_service.poll(true_state, second_robot_status,
                                Duration::fromSeconds(0.1));
    }

    const auto& first  = first_robot_status.imu_status();
    const auto& second = second_robot_status.imu_status();
    EXPECT_NE(first.angular_velocity().radians_per_second(), 0.5);
    EXPECT_NE(first.linear_acceleration().x_component_meters(), 0.0);
    EXPECT_EQ(first.angular_velocity().radians_per_second(),
              second.angular_velocity().radians_per_second());
    EXPECT_EQ(first.angular_acceleration().radians_per_second_squared(),
              second.angular_acceleration().radians_per_second_squared());
    EXPECT_EQ(first.linear_acceleration().x_component_meters(),
              second.linear_acceleration().x_component_meters());
    EXPECT_EQ(first.linear_acceleration().y_component_meters(),
              second.linear_acceleration().y_component_meters());
}

TEST_F(SimulatedImuServiceTest, reset_forgets_previous_readings)
{
    // Like the simulator moving a robot at rest to a moving state
    imu_service.poll(makeTrueState(Vector(0, 0), Angle::zero(), AngularVelocity()),
                     robot_status, Duration::fromSeconds(0.1));
    imu_service.reset();
    imu_service.poll(makeTrueState(Vector(1, 0), Angle::zero(), AngularVelocity()),
                     robot_status, Duration::fromSeconds(0.1));

    EXPECT_FALSE(robot_status.imu_status().has_angular_acceleration());
    EXPECT_FALSE(robot_status.imu_status().has_linear_acceleration());

    // Later readings are measured from the reading after the reset
    imu_service.poll(makeTrueState(Vector(1, 0), Angle::zero(), AngularVelocity()),
                     robot_status, Duration::fromSeconds(0.1));

    EXPECT_NEAR(robot_status.imu_status().linear_acceleration().x_component_meters(), 0.0,
                1e-9);
}
