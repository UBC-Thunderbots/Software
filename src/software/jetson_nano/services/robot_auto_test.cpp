#include <gtest/gtest.h>

#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/services/motor.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/message_translation/tbots_geometry.h"

class RobotAutoTestFixture : public testing::Test
{
    // Test Motor Setup and Calibration

    // Test Motor Velocities

    // See robot diagnostics on thunderscope

   protected:
    RobotAutoTestFixture()
    {
        motor_service_ = std::make_unique<MotorService>(
            create2021RobotConstants(), CONTROL_LOOP_HZ);
    }

    ~RobotAutoTestFixture() {}

    // Services
    std::unique_ptr<MotorService>
        motor_service_;  // TODO: loop_hz is not being used in motor service
};

TEST_F(RobotAutoTestFixture, SetUpMotors) {
    motor_service_->setUpMotors();
}

TEST_F(RobotAutoTestFixture, TestMotorVelocities) {
//    struct timespec iteration_time = {3, 500000000};
//    auto loop_duration =
//            iteration_time.tv_sec * static_cast<int>(NANOSECONDS_PER_SECOND) +
//            iteration_time.tv_nsec;
//    double loop_duration_seconds =
//            static_cast<double>(loop_duration) * SECONDS_PER_NANOSECOND;
//
//    int robot_id = 0;
//    PrimitiveExecutor primitive_executor = PrimitiveExecutor(CONTROL_LOOP_HZ, create2021RobotConstants(), TeamColour::YELLOW, robot_id);

    Vector expected_velocity = Vector(2, -4);
    AngularVelocity expected_angular_velocity = AngularVelocity::fromRadians(0.5);

    //TODO: we wont need to create a primitive and do the extra overhead
//    auto output = createDirectControlPrimitive(expected_velocity, expected_angular_velocity,
//                                               200, TbotsProto::AutoChipOrKick());
//
//    TbotsProto::MotorStatus motor_status_ = motor_service_->poll(output->direct_control().motor_control(), loop_duration_seconds); //TODO: what should we use for loop duration?

//    double front_right_velocity =
//            static_cast<double>(tmc4671_getActualVelocity(FRONT_RIGHT_MOTOR_CHIP_SELECT)) *
//            MECHANICAL_MPS_PER_ELECTRICAL_RPM;
//    double front_left_velocity =
//            static_cast<double>(tmc4671_getActualVelocity(FRONT_LEFT_MOTOR_CHIP_SELECT)) *
//            MECHANICAL_MPS_PER_ELECTRICAL_RPM;
//    double back_right_velocity =
//            static_cast<double>(tmc4671_getActualVelocity(BACK_RIGHT_MOTOR_CHIP_SELECT)) *
//            MECHANICAL_MPS_PER_ELECTRICAL_RPM;
//    double back_left_velocity =
//            static_cast<double>(tmc4671_getActualVelocity(BACK_LEFT_MOTOR_CHIP_SELECT)) *
//            MECHANICAL_MPS_PER_ELECTRICAL_RPM;


    //Read the Motor Status value from the motor status
//    Vector actual_local_vector = createVector(motor_status_.local_velocity());
//    AngularVelocity actual_angular_velocity = createAngularVelocity(motor_status_.angular_velocity());


}