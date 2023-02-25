#include <gtest/gtest.h>

class RobotAutoTestFixture : public testing::Test {

    // Test Motor Setup and Calibration

    // Test Motor Velocities

    // See robot diagnostics on thunderscope

protected:
    RobotAutoTestFixture() {

    }

    ~RobotAutoTestFixture() {

    }
    virtual void SetUp() {
        // Test Motor Setup and Calibration
    }

    virtual void TearDown() {
        // Take down Motors
    }

};

TEST_F(RobotAutoTestFixture, TestMotorVelocities) {

}
