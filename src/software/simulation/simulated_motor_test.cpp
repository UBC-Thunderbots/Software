#include "software/simulation/simulated_motor.h"

#include <gtest/gtest.h>

#include "proto/primitive/primitive_msg_factory.h"

class SimulatedMotorServiceTest : public ::testing::Test
{
   protected:
    // Builds a direct control primitive commanding the given local velocity
    static TbotsProto::DirectControlPrimitive makePrimitive(
        const Vector& velocity, const AngularVelocity& angular_velocity)
    {
        return createDirectControlPrimitive(velocity, angular_velocity,
                                            /*dribbler_rpm=*/0,
                                            TbotsProto::AutoChipOrKick())
            ->direct_control();
    }

    // The robot faces +x, so its local velocity equals the given global velocity
    static RobotState makeTrueState(const Vector& velocity,
                                    const AngularVelocity& angular_velocity)
    {
        return RobotState(Point(0, 0), velocity, Angle::zero(), angular_velocity);
    }

    static void expectCommandedVelocity(
        const TbotsProto::DirectControlPrimitive& primitive, const Vector& velocity,
        const AngularVelocity& angular_velocity)
    {
        const auto& direct_velocity = primitive.motor_control().direct_velocity_control();
        EXPECT_NEAR(direct_velocity.velocity().x_component_meters(), velocity.x(), 1e-9);
        EXPECT_NEAR(direct_velocity.velocity().y_component_meters(), velocity.y(), 1e-9);
        EXPECT_NEAR(direct_velocity.angular_velocity().radians_per_second(),
                    angular_velocity.toRadians(), 1e-9);
    }

    robot_constants::RobotConstants robot_constants =
        robot_constants::createRobotConstants();
    SimulatedMotorService motor_service{robot_constants, {}, 0};
    TbotsProto::RobotStatus robot_status;
};

TEST_F(SimulatedMotorServiceTest, passes_target_through_when_within_acceleration_limit)
{
    const Vector target_velocity(0.5, -0.3);
    const AngularVelocity target_angular_velocity = AngularVelocity::fromRadians(0.2);

    // A long poll interval, so the ramp doesn't limit the change
    const auto ramped =
        motor_service.poll(makePrimitive(target_velocity, target_angular_velocity),
                           makeTrueState(Vector(), AngularVelocity::zero()), robot_status,
                           Duration::fromSeconds(10));

    expectCommandedVelocity(ramped, target_velocity, target_angular_velocity);
}

TEST_F(SimulatedMotorServiceTest, ramps_when_target_exceeds_acceleration_limit)
{
    const EuclideanSpace_t target{3.0, 0.5, 1.0};
    const double time_to_ramp_s = 0.01;

    EuclideanToWheel euclidean_to_wheel(robot_constants);
    const EuclideanSpace_t expected =
        euclidean_to_wheel.getEuclideanVelocity(euclidean_to_wheel.rampWheelVelocity(
            WheelSpace_t::Zero(), euclidean_to_wheel.getWheelVelocity(target),
            time_to_ramp_s));
    ASSERT_LT(expected[0], target[0]);

    const auto ramped =
        motor_service.poll(makePrimitive(Vector(target[0], target[1]),
                                         AngularVelocity::fromRadians(target[2])),
                           makeTrueState(Vector(), AngularVelocity::zero()), robot_status,
                           Duration::fromSeconds(time_to_ramp_s));

    expectCommandedVelocity(ramped, Vector(expected[0], expected[1]),
                            AngularVelocity::fromRadians(expected[2]));
}

TEST_F(SimulatedMotorServiceTest, first_poll_ramps_from_measured_velocity)
{
    // A robot that is already moving at the commanded velocity
    const Vector velocity(1.0, 0.5);
    const AngularVelocity angular_velocity = AngularVelocity::fromRadians(0.4);

    const auto ramped = motor_service.poll(makePrimitive(velocity, angular_velocity),
                                           makeTrueState(velocity, angular_velocity),
                                           robot_status, Duration::fromSeconds(0.001));

    expectCommandedVelocity(ramped, velocity, angular_velocity);
}

TEST_F(SimulatedMotorServiceTest, later_polls_ramp_from_previous_command)
{
    // The robot doesn't move between polls, but the ramp is open loop like
    // MotorService's, so it carries on from the previous command
    const auto primitive        = makePrimitive(Vector(3.0, 0), AngularVelocity::zero());
    const RobotState true_state = makeTrueState(Vector(), AngularVelocity::zero());
    const Duration time_to_ramp = Duration::fromSeconds(0.01);

    const double first_velocity =
        motor_service.poll(primitive, true_state, robot_status, time_to_ramp)
            .motor_control()
            .direct_velocity_control()
            .velocity()
            .x_component_meters();
    const double second_velocity =
        motor_service.poll(primitive, true_state, robot_status, time_to_ramp)
            .motor_control()
            .direct_velocity_control()
            .velocity()
            .x_component_meters();

    // Each poll adds the same step, so the second command is twice the first
    EXPECT_GT(first_velocity, 0.0);
    EXPECT_NEAR(second_velocity, 2 * first_velocity, 1e-9);
}

TEST_F(SimulatedMotorServiceTest, primitive_without_velocity_command_ramps_to_stop)
{
    // PrimitiveExecutor returns an empty primitive when the robot has no primitive
    const auto ramped =
        motor_service.poll(TbotsProto::DirectControlPrimitive(),
                           makeTrueState(Vector(1.0, 0), AngularVelocity::zero()),
                           robot_status, Duration::fromSeconds(10));

    expectCommandedVelocity(ramped, Vector(), AngularVelocity::zero());
}

TEST_F(SimulatedMotorServiceTest, motor_status_reports_measured_velocity_not_command)
{
    const Vector true_velocity(0.7, -0.2);
    const AngularVelocity true_angular_velocity = AngularVelocity::fromRadians(0.3);

    motor_service.poll(makePrimitive(Vector(), AngularVelocity::zero()),
                       makeTrueState(true_velocity, true_angular_velocity), robot_status,
                       Duration::fromSeconds(10));

    const auto& motor_status = robot_status.motor_status();
    EXPECT_NEAR(motor_status.local_velocity().x_component_meters(), true_velocity.x(),
                1e-9);
    EXPECT_NEAR(motor_status.local_velocity().y_component_meters(), true_velocity.y(),
                1e-9);
    EXPECT_NEAR(motor_status.angular_velocity().radians_per_second(),
                true_angular_velocity.toRadians(), 1e-9);
    EXPECT_NEAR(motor_status.target_local_velocity().x_component_meters(), 0.0, 1e-9);
    EXPECT_NEAR(motor_status.target_local_velocity().y_component_meters(), 0.0, 1e-9);
}

TEST_F(SimulatedMotorServiceTest, passes_dribbler_and_kick_commands_through)
{
    TbotsProto::AutoChipOrKick auto_kick;
    auto_kick.set_autokick_speed_m_per_s(4.0f);
    const auto primitive =
        createDirectControlPrimitive(Vector(3.0, 0), AngularVelocity::zero(),
                                     /*dribbler_rpm=*/1000, auto_kick)
            ->direct_control();

    const auto ramped =
        motor_service.poll(primitive, makeTrueState(Vector(), AngularVelocity::zero()),
                           robot_status, Duration::fromSeconds(0.01));

    EXPECT_EQ(ramped.motor_control().dribbler_speed_rpm(), 1000);
    EXPECT_FLOAT_EQ(
        ramped.power_control().chicker().auto_chip_or_kick().autokick_speed_m_per_s(),
        4.0f);
}

TEST_F(SimulatedMotorServiceTest, same_seed_gives_same_noisy_readings)
{
    const SimulatedMotorService::NoiseConfig noise_config{.wheel_velocity_stddev_m_per_s =
                                                              0.05};
    SimulatedMotorService first_motor_service(robot_constants, noise_config, 7);
    SimulatedMotorService second_motor_service(robot_constants, noise_config, 7);

    const auto primitive        = makePrimitive(Vector(1.0, 0), AngularVelocity::zero());
    const RobotState true_state = makeTrueState(Vector(1.0, 0), AngularVelocity::zero());
    TbotsProto::RobotStatus first_robot_status;
    TbotsProto::RobotStatus second_robot_status;
    first_motor_service.poll(primitive, true_state, first_robot_status,
                             Duration::fromSeconds(0.01));
    second_motor_service.poll(primitive, true_state, second_robot_status,
                              Duration::fromSeconds(0.01));

    const double measured_velocity =
        first_robot_status.motor_status().local_velocity().x_component_meters();
    EXPECT_NE(measured_velocity, 1.0);
    EXPECT_EQ(measured_velocity,
              second_robot_status.motor_status().local_velocity().x_component_meters());
}
