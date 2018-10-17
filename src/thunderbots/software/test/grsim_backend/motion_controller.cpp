//
// Created by evan on 14/09/18.
//
#include "software/backend_output/grsim/motion_controller.h"

#include "geom/angle.h"
#include "gtest/gtest.h"
#include "shared/constants.h"
#include "software/ai/primitive/move_primitive.h"
#include "software/ai/world/robot.h"


using namespace std::chrono;


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

    double deltaTime        = 0.1;
    Point destination       = Point(0, 0);
    Angle destinationAngle  = Angle::ofDegrees(0);
    double destinationSpeed = 0;
    bool speedsEqual;

    std::pair<Vector, AngularVelocity> roboSpeeds = MotionController::grSimBangBang(
        robot, destination, destinationSpeed, destinationAngle, deltaTime);

    Vector expectedVector         = Point(0, 0);
    Angle expectedAngularVelocity = Angle::ofRadians(0);

    printf("\n%f\n", roboSpeeds.second.toDegrees());

    if (expectedVector == roboSpeeds.first &&
        expectedAngularVelocity == roboSpeeds.second)
    {
        speedsEqual = true;
    }
    else
    {
        speedsEqual = false;
    }


    EXPECT_TRUE(speedsEqual);
}

TEST_F(MotionControllerTest, calc_correct_velocity_ones)
{
    Robot robot             = Robot(2, Point(1, 0), Vector(1, 0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double deltaTime        = 1;
    Point destination       = Point(0, 0);
    Angle destinationAngle  = Angle::ofDegrees(0);
    double destinationSpeed = 0;
    bool speedsEqual;

    std::pair<Vector, AngularVelocity> roboSpeeds = MotionController::grSimBangBang(
        robot, destination, destinationSpeed, destinationAngle, deltaTime);

    Vector expectedVector         = Point(1 - ROBOT_MAX_ACCELERATION, 0);
    Angle expectedAngularVelocity = Angle::ofRadians(0);

    if (expectedVector == roboSpeeds.first &&
        expectedAngularVelocity == roboSpeeds.second)
    {
        speedsEqual = true;
    }
    else
    {
        speedsEqual = false;
    }


    EXPECT_TRUE(speedsEqual);
}

TEST_F(MotionControllerTest, over_speed_test)
{
    Robot robot       = Robot(4, Point(-1, 0), Vector(2.23, 2.23), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double deltaTime  = 1;
    Point destination = Point(5, 1);
    Angle destinationAngle  = Angle::ofDegrees(0);
    double destinationSpeed = 1.75;
    bool speedsEqual;

    std::pair<Vector, AngularVelocity> roboSpeeds = MotionController::grSimBangBang(
        robot, destination, destinationSpeed, destinationAngle, deltaTime);

    double expectedSpeed = ROBOT_MAX_SPEED;

    Angle expectedAngularVelocity = Angle::ofRadians(0);

    if (expectedSpeed == roboSpeeds.first.len() &&
        expectedAngularVelocity == roboSpeeds.second)
    {
        speedsEqual = true;
    }
    else
    {
        speedsEqual = false;
    }


    EXPECT_TRUE(speedsEqual);
}

TEST_F(MotionControllerTest, no_overspeed_acceleration_test)
{
    Robot robot            = Robot(4, Point(-1, 0), Vector(1.3, 1.3), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double deltaTime       = 1;
    Point destination      = Point(5, 1);
    Angle destinationAngle = Angle::ofDegrees(0);
    double destinationSpeed = 1.75;
    bool speedsEqual;

    std::pair<Vector, AngularVelocity> roboSpeeds = MotionController::grSimBangBang(
        robot, destination, destinationSpeed, destinationAngle, deltaTime);

    double expectedSpeed = ROBOT_MAX_SPEED;

    Angle expectedAngularVelocity = Angle::ofRadians(0);

    if (expectedSpeed == roboSpeeds.first.len() &&
        expectedAngularVelocity == roboSpeeds.second)
    {
        speedsEqual = true;
    }
    else
    {
        speedsEqual = false;
    }


    EXPECT_TRUE(speedsEqual);
}

TEST_F(MotionControllerTest, no_overspeed_ang_acceleration_test)
{
    Robot robot             = Robot(4, Point(-1, -1), Vector(0, 0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(3.9), current_time);
    double deltaTime        = 1;
    Point destination       = Point(-1, -1);
    Angle destinationAngle  = Angle::ofDegrees(210);
    double destinationSpeed = 0;
    bool speedsEqual;

    std::pair<Vector, AngularVelocity> roboSpeeds = MotionController::grSimBangBang(
        robot, destination, destinationSpeed, destinationAngle, deltaTime);

    double expectedSpeed = 0;

    Angle expectedAngularVelocity = Angle::ofRadians(4);

    if (expectedSpeed == roboSpeeds.first.len() &&
        expectedAngularVelocity == roboSpeeds.second)
    {
        speedsEqual = true;
    }
    else
    {
        speedsEqual = false;
    }


    EXPECT_TRUE(speedsEqual);
}

// TODO: make angular velocity max speed tests

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}