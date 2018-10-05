//
// Created by evan on 14/09/18.
//
#include "ai/backend_output/grsim/motion_controller.h"
#include "gtest/gtest.h"
#include "geom/angle.h"

TEST(MotionControllerTest, calc_correct_velocity_zeros)
{

    Robot robo = Robot(1U, &Point(0,0), &Point(0,0),
                                    Angle::ofDegrees(0), Angle::ofRadians(0));
    double deltaTime = 0.1;
    Point destination = Point(0,0);
    Angle destinationAngle = Angle::ofDegrees(0);
    double destinationSpeed = 1;

    std::pair<Vector, AngularVelocity> roboSpeeds = MotionController::grSim_bang_bang(robo, destination, destinationSpeed, desintationAngle, deltaTime);

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

TEST(MotionControllerTest, calc_correct_velocity_twos)
{
    Robot robo = Robot(1U, &Point(2,2), &Point(2,2),
                       Angle::ofDegrees(2), Angle::ofRadians(2));
    double deltaTime = 0.2;
    Point destination = Point(1,1);
    Angle destinationAngle = Angle::ofDegrees(22);
    double destinationSpeed = 2;

    std::pair<Vector, AngularVelocity> roboSpeeds = MotionController::grSim_bang_bang(robo, destination, destinationSpeed, desintationAngle, deltaTime);

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

int main(int argc, char** argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}