#include "grsim_communication/motion_controller.h"

#include <chrono>

#include "ai/primitive/move_primitive.h"
#include "ai/world/robot.h"
#include "geom/angle.h"
#include "gtest/gtest.h"
#include "shared/constants.h"


using namespace std::chrono;

// GrSim motion controller test file.
// Functional unit tests for the controller are located here.

#define POSITION_TOLERANCE 0.01
#define VELOCITY_BASE_TOLERANCE 0.015
#define VELOCITY_TOLERANCE_SCALE_FACTOR 0.01

#define TIME_STEP 0.001
#define TOTAL_STEPS 10000

// set up test class to keep deterministic time
class MotionControllerTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        // An arbitrary fixed point in time
        // We use this fixed point in time to make the tests deterministic.
        current_time = Timestamp::fromSeconds(300);
    }

    Timestamp current_time;

    double calculateVelocityTolerance(double velocity)
    {
        return VELOCITY_BASE_TOLERANCE + VELOCITY_TOLERANCE_SCALE_FACTOR * velocity;
    }
};

TEST_F(MotionControllerTest, calc_correct_velocity_zeros)
{
    Robot robot = Robot(1, Point(0, 0), Vector(0, 0), Angle::ofRadians(0.0),
                        AngularVelocity::ofRadians(0.0), current_time);

    double delta_time        = 0.1;
    Point destination        = Point(0, 0);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 0;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    Vector expected_vector          = Point(0, 0);
    Angle expected_angular_velocity = Angle::ofRadians(0);

    EXPECT_EQ(expected_vector, robot_velocities.linear_velocity);
    EXPECT_EQ(expected_angular_velocity, robot_velocities.angular_velocity);
}

TEST_F(MotionControllerTest, calc_correct_velocity_ones)
{
    Robot robot              = Robot(2, Point(3, 0), Vector(1, 0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time        = 1;
    Point destination        = Point(0, 0);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 0;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    Vector expected_vector =
        Point(1 - ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, 0);
    Angle expected_angular_velocity = Angle::ofRadians(0);


    EXPECT_EQ(expected_vector, robot_velocities.linear_velocity);
    EXPECT_EQ(expected_angular_velocity, robot_velocities.angular_velocity);
}

TEST_F(MotionControllerTest, over_speed_test)
{
    Robot robot       = Robot(4, Point(-1, 0), Vector(2.23, 2.23), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time = 1;
    Point destination = Point(5, 1);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 1.75;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    double expected_speed = ROBOT_MAX_SPEED_METERS_PER_SECOND;

    Angle expected_angular_velocity = Angle::ofRadians(0);

    EXPECT_DOUBLE_EQ(expected_speed, robot_velocities.linear_velocity.len());
    EXPECT_EQ(expected_angular_velocity, robot_velocities.angular_velocity);
}

TEST_F(MotionControllerTest, no_overspeed_acceleration_test)
{
    Robot robot       = Robot(4, Point(-1, 0), Vector(1.3, 1.3), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time = 1;
    Point destination = Point(5, 1);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 1.75;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    double expected_speed = ROBOT_MAX_SPEED_METERS_PER_SECOND;

    Angle expected_angular_velocity = Angle::ofRadians(0);

    EXPECT_DOUBLE_EQ(expected_speed, robot_velocities.linear_velocity.len());
    EXPECT_EQ(expected_angular_velocity, robot_velocities.angular_velocity);
}

TEST_F(MotionControllerTest, no_overspeed_ang_acceleration_test)
{
    Robot robot              = Robot(4, Point(-1, -1), Vector(0, 0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(-3.9), current_time);
    double delta_time        = 1;
    Point destination        = Point(-1, -1);
    Angle destination_angle  = Angle::ofDegrees(210);
    double destination_speed = 0;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    double expected_speed = 0;

    Angle expected_angular_speed = Angle::ofRadians(-4);

    EXPECT_DOUBLE_EQ(expected_speed, robot_velocities.linear_velocity.len());
    EXPECT_EQ(expected_angular_speed, robot_velocities.angular_velocity);
}

TEST_F(MotionControllerTest, negative_time_test)
{
    Robot robot              = Robot(4, Point(-0, -1), Vector(-1, 2), Angle::ofRadians(1),
                        AngularVelocity::ofRadians(3.9), current_time);
    double delta_time        = -1.5;
    Point destination        = Point(-1, -1);
    Angle destination_angle  = Angle::ofDegrees(210);
    double destination_speed = 0;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    Vector expected_velocity = Vector(-1, 2);

    AngularVelocity expected_angular_speed = Angle::ofRadians(3.9);

    EXPECT_EQ(expected_velocity, robot_velocities.linear_velocity);
    EXPECT_EQ(expected_angular_speed, robot_velocities.angular_velocity);
}

TEST_F(MotionControllerTest, zero_time_test)
{
    Robot robot       = Robot(4, Point(-0, -2), Vector(-1, 1.6), Angle::ofRadians(1),
                        AngularVelocity::ofRadians(3.2), current_time);
    double delta_time = 0;
    Point destination = Point(-1, -1);
    Angle destination_angle  = Angle::ofDegrees(210);
    double destination_speed = 0;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    Vector expected_velocity = Vector(-1, 1.6);

    AngularVelocity expected_angular_speed = Angle::ofRadians(3.2);

    EXPECT_EQ(expected_velocity, robot_velocities.linear_velocity);
    EXPECT_EQ(expected_angular_speed, robot_velocities.angular_velocity);
}

TEST_F(MotionControllerTest, negative_x_velocity_test)
{
    const double initialSpeed = 1;

    Robot robot = Robot(4, Point(0, 0), Vector(initialSpeed, 0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(3.9), current_time);
    double delta_time        = 1;
    Point destination        = Point(-8, 0);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 3;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    Vector expected_velocity = Vector(
        initialSpeed - ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED * delta_time, 0);

    EXPECT_EQ(expected_velocity, robot_velocities.linear_velocity);
}

TEST_F(MotionControllerTest, negative_y_velocity_test)
{
    const double initialSpeed = 1;
    Robot robot = Robot(4, Point(0, 0), Vector(0, initialSpeed), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(3.9), current_time);
    double delta_time        = 1;
    Point destination        = Point(0, -8);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 3;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    Vector expected_velocity = Vector(
        0, initialSpeed - ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED * delta_time);


    EXPECT_EQ(expected_velocity, robot_velocities.linear_velocity);
}

TEST_F(MotionControllerTest, negative_desired_orientation_test)
{
    double delta_time                 = 1;
    Point destination                 = Point(0, 0);
    Angle destination_angle           = Angle::ofDegrees(-179);
    double destination_speed          = 0;
    AngularVelocity initial_ang_speed = AngularVelocity::ofRadians(3);

    Robot robot = Robot(4, Point(0, 0), Vector(0, 0), Angle::ofRadians(0),
                        initial_ang_speed, current_time);


    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    // expect -4 because of angular speed cap
    AngularVelocity expected_velocity =
        AngularVelocity::ofRadians(-ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND);

    EXPECT_EQ(expected_velocity, robot_velocities.angular_velocity);
}

TEST_F(MotionControllerTest, positive_desired_orientation_test)
{
    double delta_time        = 1;
    Point destination        = Point(0, 0);
    Angle destination_angle  = Angle::ofDegrees(179);
    double destination_speed = 0;

    AngularVelocity initial_ang_speed = AngularVelocity::ofRadians(-3);

    Robot robot = Robot(4, Point(0, 0), Vector(0, 0), Angle::ofRadians(0),
                        initial_ang_speed, current_time);

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    // expect -4 because of angular speed cap
    AngularVelocity expected_velocity =
        AngularVelocity::ofRadians(ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND);

    EXPECT_EQ(expected_velocity, robot_velocities.angular_velocity);
}

TEST_F(MotionControllerTest, negative_y_positive_x_velocity_test)
{
    double delta_time        = 1;
    Point destination        = Point(2, -2);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 6;

    Robot robot = Robot(4, Point(1, -1), Vector(0, 0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    Vector expected_velocity = Vector(sqrt(ROBOT_MAX_SPEED_METERS_PER_SECOND),
                                      -sqrt(ROBOT_MAX_SPEED_METERS_PER_SECOND));

    EXPECT_EQ(expected_velocity, robot_velocities.linear_velocity);
}

TEST_F(MotionControllerTest, positive_y_negative_x_velocity_test)
{
    Robot robot              = Robot(4, Point(-1, 1), Vector(0, 0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time        = 1;
    Point destination        = Point(-2, 2);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 6;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    Vector expected_velocity = Vector(-sqrt(ROBOT_MAX_SPEED_METERS_PER_SECOND),
                                      sqrt(ROBOT_MAX_SPEED_METERS_PER_SECOND));

    EXPECT_EQ(expected_velocity, robot_velocities.linear_velocity);
}

TEST_F(MotionControllerTest, positive_y_positive_x_velocity_test)
{
    Robot robot              = Robot(4, Point(1, 1), Vector(0, 0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time        = 1;
    Point destination        = Point(2, 2);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 6;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    Vector expected_velocity = Vector(sqrt(ROBOT_MAX_SPEED_METERS_PER_SECOND),
                                      sqrt(ROBOT_MAX_SPEED_METERS_PER_SECOND));

    EXPECT_EQ(expected_velocity, robot_velocities.linear_velocity);
}

TEST_F(MotionControllerTest, negative_y_negative_x_velocity_test)
{
    Robot robot       = Robot(4, Point(-1.5, -1.5), Vector(0, 0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time = 1;
    Point destination = Point(-3, -3);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 6;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    Vector expected_velocity = Vector(-sqrt(ROBOT_MAX_SPEED_METERS_PER_SECOND),
                                      -sqrt(ROBOT_MAX_SPEED_METERS_PER_SECOND));

    EXPECT_EQ(expected_velocity, robot_velocities.linear_velocity);
}

TEST_F(MotionControllerTest, zero_final_speed_positive_x_positive_y_position_test)
{
    Robot robot = Robot(4, Point(3.0, 3.0), Vector(0.0, 0.0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    const double delta_time  = TIME_STEP;
    Point destination        = Point(3.05, 3.05);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 0;

    int iteration_count;
    MotionController::Velocity robot_velocities;
    double new_x_position;
    double new_y_position;

    for (iteration_count = 0; iteration_count < TOTAL_STEPS; iteration_count++)
    {
        robot_velocities = MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

        new_x_position = robot.position().x() + robot.velocity().x() * delta_time;
        new_y_position = robot.position().y() + robot.velocity().y() * delta_time;

        Robot tempRobo = Robot(4, Point(new_x_position, new_y_position),
                               robot_velocities.linear_velocity, robot.orientation(),
                               robot_velocities.angular_velocity, current_time);
        robot.updateState(tempRobo);
    }


    Vector expected_velocity = Vector(0, 0);


    EXPECT_NEAR(robot.position().x(), destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.position().y(), destination.y(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.velocity().x(), expected_velocity.x(),
                calculateVelocityTolerance(destination_speed));
    EXPECT_NEAR(robot.velocity().y(), expected_velocity.y(),
                calculateVelocityTolerance(destination_speed));
}

TEST_F(MotionControllerTest, zero_final_speed_positive_x_negative_y_position_test)
{
    Robot robot       = Robot(4, Point(3.0, 0.05), Vector(0.0, 0.0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time = TIME_STEP;
    Point destination = Point(3.05, -0.02);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 0;
    int iteration_count;
    MotionController::Velocity robot_velocities;
    double new_x_position;
    double new_y_position;
    Vector expected_velocity = Vector(0, 0);

    for (iteration_count = 0; iteration_count < TOTAL_STEPS; iteration_count++)
    {
        robot_velocities = MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

        new_x_position = robot.position().x() + robot.velocity().x() * delta_time;
        new_y_position = robot.position().y() + robot.velocity().y() * delta_time;

        Robot tempRobo = Robot(4, Point(new_x_position, new_y_position),
                               robot_velocities.linear_velocity, robot.orientation(),
                               robot_velocities.angular_velocity, current_time);
        robot.updateState(tempRobo);
    }

    EXPECT_NEAR(robot.position().x(), destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.position().y(), destination.y(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.velocity().x(), expected_velocity.x(),
                calculateVelocityTolerance(destination_speed));
    EXPECT_NEAR(robot.velocity().y(), expected_velocity.y(),
                calculateVelocityTolerance(destination_speed));
}

TEST_F(MotionControllerTest, zero_final_speed_negative_x_positive_y_position_test)
{
    Robot robot       = Robot(4, Point(0.05, 3.0), Vector(0.0, 0.0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time = TIME_STEP;
    Point destination = Point(-0.025, 3.05);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 0;
    int iteration_count;
    MotionController::Velocity robot_velocities;
    double new_x_position;
    double new_y_position;

    for (iteration_count = 0; iteration_count < TOTAL_STEPS; iteration_count++)
    {
        robot_velocities = MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

        new_x_position = robot.position().x() + robot.velocity().x() * delta_time;
        new_y_position = robot.position().y() + robot.velocity().y() * delta_time;

        robot.updateState(Point(new_x_position, new_y_position),
                          robot_velocities.linear_velocity, robot.orientation(),
                          robot_velocities.angular_velocity, current_time);
    }


    Vector expected_velocity = Vector(0, 0);


    EXPECT_NEAR(robot.position().x(), destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.position().y(), destination.y(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.velocity().x(), expected_velocity.x(),
                calculateVelocityTolerance(destination_speed));
    EXPECT_NEAR(robot.velocity().y(), expected_velocity.y(),
                calculateVelocityTolerance(destination_speed));
}

TEST_F(MotionControllerTest, zero_final_speed_negative_x_negative_y_position_test)
{
    Robot robot       = Robot(4, Point(0.05, 0.05), Vector(0.0, 0.0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time = TIME_STEP;
    Point destination = Point(-0.025, -0.025);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 0;
    int iteration_count;
    MotionController::Velocity robot_velocities;
    double new_x_position;
    double new_y_position;

    for (iteration_count = 0; iteration_count < TOTAL_STEPS; iteration_count++)
    {
        robot_velocities = MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

        new_x_position = robot.position().x() + robot.velocity().x() * delta_time;
        new_y_position = robot.position().y() + robot.velocity().y() * delta_time;

        robot.updateState(Point(new_x_position, new_y_position),
                          robot_velocities.linear_velocity, robot.orientation(),
                          robot_velocities.angular_velocity, current_time);
    }


    Vector expected_velocity = Vector(0, 0);


    EXPECT_NEAR(robot.position().x(), destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.position().y(), destination.y(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.velocity().x(), expected_velocity.x(),
                calculateVelocityTolerance(destination_speed));
    EXPECT_NEAR(robot.velocity().y(), expected_velocity.y(),
                calculateVelocityTolerance(destination_speed));
}

TEST_F(MotionControllerTest, asymetric_reach_des_test)
{
    Robot robot       = Robot(4, Point(0.08, 0.05), Vector(0.0, 0.0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time = TIME_STEP;
    Point destination = Point(-1.25, -0.78);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 0;
    int iteration_count;
    MotionController::Velocity robot_velocities;
    double new_x_position;
    double new_y_position;

    for (iteration_count = 0; iteration_count < TOTAL_STEPS; iteration_count++)
    {
        robot_velocities = MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

        new_x_position = robot.position().x() + robot.velocity().x() * delta_time;
        new_y_position = robot.position().y() + robot.velocity().y() * delta_time;

        robot.updateState(Point(new_x_position, new_y_position),
                          robot_velocities.linear_velocity, robot.orientation(),
                          robot_velocities.angular_velocity, current_time);
    }


    Vector expected_velocity = Vector(0, 0);


    EXPECT_NEAR(robot.position().x(), destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.position().y(), destination.y(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.velocity().x(), expected_velocity.x(),
                calculateVelocityTolerance(destination_speed));
    EXPECT_NEAR(robot.velocity().y(), expected_velocity.y(),
                calculateVelocityTolerance(destination_speed));
}

TEST_F(MotionControllerTest, positive_final_speed_position_test)
{
    Robot robot             = Robot(4, Point(0, 0), Vector(0.0, 0.0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time       = TIME_STEP;
    Point destination       = Point(1., 1);
    Angle destination_angle = Angle::ofDegrees(0);
    double destination_speed = 1;
    int iteration_count;
    MotionController::Velocity robot_velocities;
    double new_x_position;
    double new_y_position;
    double position_epsilon  = 0.01;
    Vector expected_velocity = Vector(0.74, 0.74);

    for (iteration_count = 0; iteration_count < TOTAL_STEPS; iteration_count++)
    {
        robot_velocities = MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

        new_x_position = robot.position().x() + robot.velocity().x() * delta_time;
        new_y_position = robot.position().y() + robot.velocity().y() * delta_time;

        robot.updateState(Point(new_x_position, new_y_position),
                          robot_velocities.linear_velocity, robot.orientation(),
                          robot_velocities.angular_velocity, current_time);

        if ((destination - robot.position()).len() < position_epsilon)
        {
            break;
        }
    }

    EXPECT_NEAR(robot.position().x(), destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.position().y(), destination.y(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.velocity().x(), expected_velocity.x(),
                calculateVelocityTolerance(destination_speed));
    EXPECT_NEAR(robot.velocity().y(), expected_velocity.y(),
                calculateVelocityTolerance(destination_speed));
}

TEST_F(MotionControllerTest, negative_final_speed_position_test)
{
    Robot robot             = Robot(4, Point(0, 0), Vector(0.0, 0.0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time       = TIME_STEP;
    Point destination       = Point(-1., -1);
    Angle destination_angle = Angle::ofDegrees(0);
    double destination_speed = 1;
    int iteration_count;
    MotionController::Velocity robot_velocities;
    double new_x_position;
    double new_y_position;
    double position_epsilon  = 0.01;
    Vector expected_velocity = Vector(-0.74, -0.74);

    for (iteration_count = 0; iteration_count < TOTAL_STEPS; iteration_count++)
    {
        robot_velocities = MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

        new_x_position = robot.position().x() + robot.velocity().x() * delta_time;
        new_y_position = robot.position().y() + robot.velocity().y() * delta_time;

        robot.updateState(Point(new_x_position, new_y_position),
                          robot_velocities.linear_velocity, robot.orientation(),
                          robot_velocities.angular_velocity, current_time);

        if ((destination - robot.position()).len() < position_epsilon)
        {
            break;
        }
    }

    EXPECT_NEAR(robot.position().x(), destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.position().y(), destination.y(), POSITION_TOLERANCE);
    EXPECT_NEAR(robot.velocity().x(), expected_velocity.x(),
                calculateVelocityTolerance(destination_speed));
    EXPECT_NEAR(robot.velocity().y(), expected_velocity.y(),
                calculateVelocityTolerance(destination_speed));
}

TEST_F(MotionControllerTest, positive_rotation_position_test)
{
    Robot robot             = Robot(4, Point(0, 0), Vector(0.0, 0.0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    const double delta_time = TIME_STEP;
    Point destination       = Point(0., 0);
    Angle destination_angle = Angle::ofDegrees(90);
    double destination_speed = 0;
    int iteration_count;
    MotionController::Velocity robot_velocities;
    Angle new_orientation;
    AngularVelocity final_angular_speed = AngularVelocity::ofRadians(0);

    for (iteration_count = 0; iteration_count < TOTAL_STEPS; iteration_count++)
    {
        robot_velocities = MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

        new_orientation =
            robot.orientation() +
            Angle::ofRadians(robot.angularVelocity().toRadians() * delta_time);

        Robot tempRobo =
            Robot(4, robot.position(), robot_velocities.linear_velocity, new_orientation,
                  robot_velocities.angular_velocity, current_time);
        robot.updateState(robot.position(), robot_velocities.linear_velocity,
                          new_orientation, robot_velocities.angular_velocity,
                          current_time);
    }

    EXPECT_NEAR(robot.orientation().toRadians(), destination_angle.toRadians(),
                POSITION_TOLERANCE);
    EXPECT_NEAR(robot.angularVelocity().toRadians(), final_angular_speed.toRadians(),
                POSITION_TOLERANCE);
}

TEST_F(MotionControllerTest, negative_rotation_position_test)
{
    Robot robot             = Robot(4, Point(0, 0), Vector(0.0, 0.0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    const double delta_time = TIME_STEP;
    Point destination       = Point(0., 0);
    Angle destination_angle = Angle::ofRadians(-1);
    double destination_speed = 0;
    int iteration_count;
    MotionController::Velocity robot_velocities;
    Angle new_orientation;
    AngularVelocity final_angular_speed = AngularVelocity::ofRadians(0);

    for (iteration_count = 0; iteration_count < TOTAL_STEPS; iteration_count++)
    {
        robot_velocities = MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

        new_orientation =
            robot.orientation() +
            Angle::ofRadians(robot.angularVelocity().toRadians() * delta_time);

        robot.updateState(robot.position(), robot_velocities.linear_velocity,
                          new_orientation, robot_velocities.angular_velocity,
                          current_time);
    }

    EXPECT_NEAR(robot.orientation().toRadians(), destination_angle.toRadians(),
                POSITION_TOLERANCE);
    EXPECT_NEAR(robot.angularVelocity().toRadians(), final_angular_speed.toRadians(),
                POSITION_TOLERANCE);
}

TEST_F(MotionControllerTest,
       wrong_direction_impulse_while_close_to_destination_small_initial_velocity_test)
{
    // basic sanity test to make sure we don't continue accelerating in the wrong
    // direction when we accelerate slightly in the opposite direction of the destination
    // https://github.com/UBC-Thunderbots/Software/issues/270

    // we need to test the case that we have a small velocity away from the destination
    // while we are trying to accelerate toward the destination
    Vector initial_velocity(-0.1, 0);
    Robot robot             = Robot(0, Point(0, 0), initial_velocity, Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    const double delta_time = TIME_STEP;
    Point destination       = Point(0.1, 0);
    Angle destination_angle = Angle::ofRadians(0);

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(robot, destination, 2,
                                                     destination_angle, delta_time);
    // if the destination is on the +x side of the robot, the velocity should always have
    // x > 0
    EXPECT_GT(robot_velocities.linear_velocity.x(), initial_velocity.x());
}

TEST_F(MotionControllerTest,
       wrong_direction_impulse_while_close_to_destination_large_initial_velocity_test)
{
    // same as above, test with a large initial velocity this time
    Vector initial_velocity(-2, 0);
    Robot robot             = Robot(0, Point(0, 0), initial_velocity, Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    const double delta_time = TIME_STEP;
    Point destination       = Point(0.1, 0);
    Angle destination_angle = Angle::ofRadians(0);

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(robot, destination, 2,
                                                     destination_angle, delta_time);
    // if the destination is on the +x side of the robot, the velocity should always have
    // x > 0
    EXPECT_GT(robot_velocities.linear_velocity.x(), initial_velocity.x());
}
