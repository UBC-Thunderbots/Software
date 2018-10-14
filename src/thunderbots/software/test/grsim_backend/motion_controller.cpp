//
// Created by evan on 14/09/18.
//
#include "software/backend_output/grsim/motion_controller.h"
#include "gtest/gtest.h"
#include "geom/angle.h"
#include "software/ai/world/robot.h"

using namespace std::chrono;

class MotionControllerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        auto epoch       = time_point<std::chrono::steady_clock>();
        auto since_epoch = std::chrono::seconds(10000);

        // An arbitrary fixed point in time. 10000 seconds after the epoch.
        // We use this fixed point in time to make the tests deterministic.
        current_time       = epoch + since_epoch;
        half_second_future = current_time + milliseconds(500);
        one_second_future  = current_time + seconds(1);
    }

    steady_clock::time_point current_time;
    steady_clock::time_point half_second_future;
    steady_clock::time_point one_second_future;
};

TEST_F(MotionControllerTest, calc_correct_velocity_zeros)
{

    Robot robot = Robot(3, Point(1, 1), Vector(-0.3, 0), Angle::ofRadians(2.2),
                        AngularVelocity::ofRadians(-0.6), current_time);
    double deltaTime = 0.1;
    Point destination = Point(0,0);
    Angle destinationAngle = Angle::ofDegrees(0);
    double destinationSpeed = 1;
    bool speedsEqual;

    std::pair<Vector, AngularVelocity> roboSpeeds = MotionController::grSim_bang_bang(robot, destination, destinationSpeed, destinationAngle, deltaTime);

    Vector expectedVector = Point(0,0);
    Angle expectedAngularVelocity = Angle::ofRadians(0);

    if (expectedVector == roboSpeeds.first && expectedAngularVelocity == roboSpeeds.second) {
        speedsEqual = true;
    }
    else {
        speedsEqual = false;
    }


    EXPECT_TRUE(speedsEqual);
}

//TEST_F(MotionControllerTest, calc_correct_velocity_twos)
//{
//    Robot robo = Robot(1U, &Point(2,2), &Point(2,2),
//                       Angle::ofDegrees(2), Angle::ofRadians(2));
//    double deltaTime = 0.2;
//    Point destination = Point(1,1);
//    Angle destinationAngle = Angle::ofDegrees(22);
//    double destinationSpeed = 2;(0,0);
//    Angle expectedAngularVelocity = Angle::ofRadians(0);
//
//
//    std::pair<Vector, AngularVelocity> roboSpeeds = MotionController::grSim_bang_bang(robo, destination, destinationSpeed, desintationAngle, deltaTime);
//
//    Vector expectedVector = Point;
//    if (expectedVector == roboSpeeds.first && expectedAngularVelocity == roboSpeeds.second) {
//    speedsEqual = true;
//    }
//    else {
//    speedsEqual = false;
//    }
//
//
//    EXPECT_TRUE(speedsEqual);
//
//}

//int main(int argc, char **argv)
//{
//    std::cout << argv[0] << std::endl;
//    testing::InitGoogleTest(&argc, argv);
//    return RUN_ALL_TESTS();
//}