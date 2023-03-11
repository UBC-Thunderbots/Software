#include <gtest/gtest.h>

#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/services/motor.h"
#include "software/jetson_nano/primitive_executor.h"

class RobotAutoTestFixture : public testing::Test
{
    // Test Motor Setup and Calibration

    // Test Motor Velocities

    // See robot diagnostics on thunderscope

   protected:
    RobotAutoTestFixture()
    {
        motor_service = std::make_unique<MotorService>(
            create2021RobotConstants(), CONTROL_LOOP_HZ);
    }

    ~RobotAutoTestFixture() {}

    // Services
    std::unique_ptr<MotorService>
        motor_service;  // TODO: loop_hz is not being used in motor service
};

TEST_F(RobotAutoTestFixture, SetUpMotors) {
    motor_service->setUpMotors();
}

TEST_F(RobotAutoTestFixture, TestMotorVelocities) {
    int robot_id = 0; //std::stoi(redis_client_->getSync(ROBOT_ID_REDIS_KEY))
    PrimitiveExecutor primitive_executor = PrimitiveExecutor(CONTROL_LOOP_HZ, create2021RobotConstants(), TeamColour::YELLOW, robot_id);
    TbotsProto::MotorStatus motor_status = motor_service->poll(primitive_executor.stepPrimitive()->motor_control(), ); //TODO: what should we use for loop duration?

    //Read the Motor Status value
}