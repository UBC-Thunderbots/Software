//
// Created by evan on 14/09/18.
//
#include "software/backend_output/grsim/motion_controller.h"
#include "gtest/gtest.h"
#include "geom/angle.h"
#include "software/ai/world/robot.h"
#include "software/ai/primitive/move_primitive.h"

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

//TEST_F(MotionControllerTest, calc_correct_velocity_zeros)
//{
//
//    Robot robot = Robot(1, Point(0, 0), Vector(0, 0), Angle::ofRadians(0.0),
//                        AngularVelocity::ofRadians(0.0), current_time);
//    double deltaTime = 0.1;
//    Point destination = Point(0,0);
//    Angle destinationAngle = Angle::ofDegrees(0);
//    double destinationSpeed = 0;
//    bool speedsEqual;
//
//    std::pair<Vector, AngularVelocity> roboSpeeds = MotionController::grSim_bang_bang(robot, destination, destinationSpeed, destinationAngle, deltaTime);
//
//    Vector expectedVector = Point(0,0);
//    Angle expectedAngularVelocity = Angle::ofRadians(0);
//
//    printf("\n%f\n", roboSpeeds.second.toDegrees());
//
//    if (expectedVector == roboSpeeds.first && expectedAngularVelocity == roboSpeeds.second) {
//        speedsEqual = true;
//    }
//    else {
//        speedsEqual = false;
//    }
//
//
//    EXPECT_TRUE(speedsEqual);
//}

TEST_F(MotionControllerTest, calc_correct_velocity_ones)
{

    Robot robot = Robot(2, Point(1, 0), Vector(1, 0), Angle::ofRadians(0),
                        AngularVelocity::ofRadians(0), current_time);
    double deltaTime = 1;
    Point destination = Point(0,0);
    Angle destinationAngle = Angle::ofDegrees(0);
    double destinationSpeed = 0;
    bool speedsEqual;

    std::pair<Vector, AngularVelocity> roboSpeeds = MotionController::grSim_bang_bang(robot, destination, destinationSpeed, destinationAngle, deltaTime);

    Vector expectedVector = Point(0,0);
    Angle expectedAngularVelocity = Angle::ofRadians(0);

    printf("\n%f\n", roboSpeeds.second.toDegrees());

    if (expectedVector == roboSpeeds.first && expectedAngularVelocity == roboSpeeds.second) {
        speedsEqual = true;
    }
    else {
        speedsEqual = false;
    }


    EXPECT_TRUE(speedsEqual);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}