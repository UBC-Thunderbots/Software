#include "software/embedded/robot_localizer.h"

#include <gtest/gtest.h>

TEST(BasicTest, basic_test2) {
    RobotLocalizer localizer(0.007, 0.007, 0.5);
    localizer.UpdateEncoders(AngularVelocity::fromDegrees(34));
    usleep(1000*1000);
    localizer.Step(AngularVelocity::zero());
    usleep(1000*1000);
    localizer.Step(AngularVelocity::zero());
    usleep(1000*1000);
    localizer.Step(AngularVelocity::zero());
    localizer.RollbackVision(Angle::fromDegrees(45), 1.1);
    localizer.RollbackVision(Angle::fromDegrees(45*2), 0.3);
    std::cout << "Thought:" << std::endl;
    std::cout << localizer.getOrientation().toRadians() << std::endl;
    std::cout << localizer.getAngularVelocity().toRadians() << std::endl;
}

TEST(BasicTest, basic_test) {
    // no variance
    RobotLocalizer localizer(0.007, 0.007, 0.5);
    usleep(3000);
    localizer.UpdateEncoders(AngularVelocity::fromDegrees(34));
    localizer.RollbackVision(Angle::fromDegrees(45), 0);
    std::cout << "Thought:" << std::endl;
    std::cout << localizer.getOrientation().toRadians() << std::endl;
    std::cout << localizer.getAngularVelocity().toRadians() << std::endl;
    usleep(1000*1000);
    localizer.Step(AngularVelocity::zero());
    localizer.RollbackVision(Angle::fromDegrees(45*2), 0);
    std::cout << "Thought:" << std::endl;
    std::cout << localizer.getOrientation().toRadians() << std::endl;
    std::cout << localizer.getAngularVelocity().toRadians() << std::endl;
    usleep(1000*1000);
    localizer.Step(AngularVelocity::zero());
    localizer.RollbackVision(Angle::fromDegrees(45*3), 0);
    std::cout << "Thought:" << std::endl;
    std::cout << localizer.getOrientation().toRadians() << std::endl;
    std::cout << localizer.getAngularVelocity().toRadians() << std::endl;
    usleep(1000*1000);
    localizer.Step(AngularVelocity::zero());
    localizer.RollbackVision(Angle::fromDegrees(45*4), 0);
    std::cout << "Thought:" << std::endl;
    std::cout << localizer.getOrientation().toRadians() << std::endl;
    std::cout << localizer.getAngularVelocity().toRadians() << std::endl;
    usleep(1000*1000);
    localizer.Step(AngularVelocity::zero());
    localizer.RollbackVision(Angle::fromDegrees(45*5), 0);
    std::cout << "Thought:" << std::endl;
    std::cout << localizer.getOrientation().toRadians() << std::endl;
    std::cout << localizer.getAngularVelocity().toRadians() << std::endl;
    std::cout << localizer.getOrientation().toRadians() << std::endl;
    std::cout << localizer.getAngularVelocity().toRadians() << std::endl;
    ASSERT_TRUE(abs(localizer.getAngularVelocity().toDegrees() - 4.0) < 0.01);
}
