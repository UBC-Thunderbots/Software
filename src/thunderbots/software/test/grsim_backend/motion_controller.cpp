#include "software/backend_output/grsim/motion_controller.h"

#include "geom/angle.h"
#include "gtest/gtest.h"
#include "shared/constants.h"
#include "software/ai/primitive/move_primitive.h"
#include "software/ai/world/robot.h"


using namespace std::chrono;

// GrSim motion controller test file.
// Functional unit tests for the controller are located here.


// set up test class to keep deterministic time
class MotionControllerTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        auto epoch       = time_point<std::chrono::steady_clock>();
        auto since_epoch = std::chrono::seconds(10000);

        // An arbitrary fixed point in time. 10000 seconds after the epoch.
        // We use this fixed point in time to make the tests deterministic.
        current_time = epoch + since_epoch;
    }

    steady_clock::time_point current_time;
};

TEST_F(MotionControllerTest, calc_correct_velocity_zeros)
{
    Robot robot = Robot(1, Point(0, 0), Vector(0, 0), Angle::ofRadians(0.0),
                        AngularVelocity::ofRadians(0.0), current_time);

    double delta_time        = 0.1;
    Point destination        = Point(0, 0);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 0;
    bool speeds_equal;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    Vector expected_vector          = Point(0, 0);
    Angle expected_angular_velocity = Angle::ofRadians(0);

    if (expected_vector == robot_velocities.linear_velocity &&
        expected_angular_velocity == robot_velocities.angular_velocity)
    {
        speeds_equal = true;
    }
    else
    {
        speeds_equal = false;
    }


    EXPECT_TRUE(speeds_equal);
}

TEST_F(MotionControllerTest, calc_correct_velocity_ones)
{
    Robot robot              = Robot(2, Point(1, 0), Vector(1, 0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time        = 1;
    Point destination        = Point(0, 0);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 0;
    bool speeds_equal;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    Vector expected_vector          = Point(1 - ROBOT_MAX_ACCELERATION, 0);
    Angle expected_angular_velocity = Angle::ofRadians(0);

    if (expected_vector == robot_velocities.linear_velocity &&
        expected_angular_velocity == robot_velocities.angular_velocity)
    {
        speeds_equal = true;
    }
    else
    {
        speeds_equal = false;
    }


    EXPECT_TRUE(speeds_equal);
}

TEST_F(MotionControllerTest, over_speed_test)
{
    Robot robot       = Robot(4, Point(-1, 0), Vector(2.23, 2.23), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time = 1;
    Point destination = Point(5, 1);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 1.75;
    bool speeds_equal;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    double expected_speed = ROBOT_MAX_SPEED;

    Angle expected_angular_velocity = Angle::ofRadians(0);

    if (expected_speed == robot_velocities.linear_velocity.len() &&
        expected_angular_velocity == robot_velocities.angular_velocity)
    {
        speeds_equal = true;
    }
    else
    {
        speeds_equal = false;
    }


    EXPECT_TRUE(speeds_equal);
}

TEST_F(MotionControllerTest, no_overspeed_acceleration_test)
{
    Robot robot       = Robot(4, Point(-1, 0), Vector(1.3, 1.3), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double delta_time = 1;
    Point destination = Point(5, 1);
    Angle destination_angle  = Angle::ofDegrees(0);
    double destination_speed = 1.75;
    bool speeds_equal;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    double expectedSpeed = ROBOT_MAX_SPEED;

    Angle expectedAngularVelocity = Angle::ofRadians(0);

    if (expectedSpeed == robot_velocities.linear_velocity.len() &&
        expectedAngularVelocity == robot_velocities.angular_velocity)
    {
        speeds_equal = true;
    }
    else
    {
        speeds_equal = false;
    }


    EXPECT_TRUE(speeds_equal);
}

TEST_F(MotionControllerTest, no_overspeed_ang_acceleration_test)
{
    Robot robot              = Robot(4, Point(-1, -1), Vector(0, 0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(3.9), current_time);
    double delta_time        = 1;
    Point destination        = Point(-1, -1);
    Angle destination_angle  = Angle::ofDegrees(210);
    double destination_speed = 0;
    bool speeds_equal;

    MotionController::Velocity robot_velocities =
        MotionController::bangBangVelocityController(
            robot, destination, destination_speed, destination_angle, delta_time);

    double expected_speed = 0;

    Angle expected_angular_speed = Angle::ofRadians(4);

    if (expected_speed == robot_velocities.linear_velocity.len() &&
        expected_angular_speed == robot_velocities.angular_velocity)
    {
        speeds_equal = true;
    }
    else
    {
        speeds_equal = false;
    }


    EXPECT_TRUE(speeds_equal);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
