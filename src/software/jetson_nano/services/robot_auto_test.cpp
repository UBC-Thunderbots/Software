#include <gtest/gtest.h>

#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/services/motor.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/message_translation/tbots_geometry.h"
#include <chrono>
#include <thread>

extern "C"
{
#include "external/trinamic/tmc/ic/TMC4671/TMC4671.h"
#include "external/trinamic/tmc/ic/TMC4671/TMC4671_Variants.h"
#include "external/trinamic/tmc/ic/TMC6100/TMC6100.h"
}

class RobotAutoTestFixture : public testing::Test
{
    // Test Motor Setup and Calibration

    // Test Motor Velocities

    // See robot diagnostics on thunderscope

   protected:
    void SetUp(void) {
        motor_service_->setUpMotors();

    }

    void TearDown(void) {
        // reset the motor velocity
        for (uint8_t motor = 0; motor < NUM_DRIVE_MOTORS; motor++)
        {
            motor_service_->setTargetVelocity(motor, 0);
        }
    }

    RobotAutoTestFixture()
    {
        motor_service_ = std::make_unique<MotorService>(
            create2021RobotConstants(), CONTROL_LOOP_HZ);
        robot_constants_ = create2021RobotConstants();
    }

    ~RobotAutoTestFixture() {}

    // Services
    std::unique_ptr<MotorService>
        motor_service_;  // TODO: loop_hz is not being used in motor service
    RobotConstants_t robot_constants_;

    // SPI Chip Selects
    static const uint8_t FRONT_LEFT_MOTOR_CHIP_SELECT  = 0;
    static const uint8_t FRONT_RIGHT_MOTOR_CHIP_SELECT = 3;
    static const uint8_t BACK_LEFT_MOTOR_CHIP_SELECT   = 1;
    static const uint8_t BACK_RIGHT_MOTOR_CHIP_SELECT  = 2;
    static const uint8_t NUM_DRIVE_MOTORS              = 4;
};

TEST_F(RobotAutoTestFixture, TestFrontRightMotorVelocity) {

    // 1. Check driver fault, make the function public
    if (motor_service_->checkDriverFault(FRONT_RIGHT_MOTOR_CHIP_SELECT)) {
        LOG(FATAL) << "Detected Motor Fault";
    }

    // 3. Abstract out drive motor setup into a function
    motor_service_->setUpDriveMotor(FRONT_RIGHT_MOTOR_CHIP_SELECT);

    // 2. Log messages for motor calibration
    motor_service_->startEncoderCalibration(FRONT_RIGHT_MOTOR_CHIP_SELECT);
    LOG(INFO) << "Starting motor encoder calibration";
    motor_service_->endEncoderCalibration(FRONT_RIGHT_MOTOR_CHIP_SELECT);
    LOG(INFO) << "Ending motor encoder calibration";

    WheelSpace_t prev_wheel_velocities = {0.0, 0.0, 0.0, 0.0};
    WheelSpace_t target_wheel_velocities = {0.0, 0.0, 0.0, 0.0};
    EuclideanSpace_t target_linear_velocity  = {0.1, 0.1, 0.0};

    for (int i = 0; i < 10; i++) {
        target_wheel_velocities = motor_service_->rampWheelVelocity(prev_wheel_velocities, target_linear_velocity, static_cast<double>(robot_constants_.robot_max_speed_m_per_s),
                                                                    static_cast<double>(robot_constants_.robot_max_acceleration_m_per_s_2), 0.1);
        prev_wheel_velocities = target_wheel_velocities;

        // Set target speeds accounting for acceleration
        int write_value = static_cast<int>(target_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX] * MotorService::ELECTRICAL_RPM_PER_MECHANICAL_MPS);
        motor_service_->writeIntToTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_TARGET, write_value);

        int read_value = motor_service_->readIntFromTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_TARGET);

        if (write_value != read_value) {
            FAIL() << "Read and write value are different";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Read the Motor Status value from the motor status
        double front_right_velocity =
                static_cast<double>(motor_service_->readVelocityFromTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT));
        LOG(INFO) << target_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX] << "Target wheel velocity";
        LOG(INFO) << front_right_velocity << "Actual wheel velocity read";

        // Run-away protection
        motor_service_->disableVelocity(front_right_velocity, prev_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX]);


        // 4. Make the threshold more lenient for expect equal double
        //EXPECT_DOUBLE_EQ(front_right_velocity, target_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX]);
        ASSERT_NEAR(front_right_velocity, target_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX], 0.01);

        // 5. Check driver faults again after making updates to motor
        motor_service_->checkDriverFault(FRONT_RIGHT_MOTOR_CHIP_SELECT);

        // 6. Detect the reset than
        int reset_detector = motor_service_->readIntFromTMC4671(0, TMC4671_PID_ACCELERATION_LIMIT);

        // When the motor board is reset the value in the above register is set to the maximum
        // signed 32 bit value Please read the header file and the datasheet for more info
        if (reset_detector == 2147483647)
        {
            FAIL() << "RESET DETECTED";
        }
    }

    // reset the motor velocity
    for (uint8_t motor = 0; motor < NUM_DRIVE_MOTORS; motor++)
    {
        motor_service_->setTargetVelocity(motor, 0);
    }

}

//TEST_F(RobotAutoTestFixture, TestFrontRightMotorNoise) {
//
//    //Checking for noise
//    double front_right_velocity1 =
//            static_cast<double>(motor_service_->readVelocityFromTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT));
//    LOG(INFO) << front_right_velocity1 << ": Reading velocity calculation 1";
//
//    // Wait for two seconds
//    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//
//    // Checking for noise again
//    double front_right_velocity2 =
//            static_cast<double>(motor_service_->readVelocityFromTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT));
//    LOG(INFO) << front_right_velocity2 << ": Reading velocity calculation 2";
//
//    EXPECT_DOUBLE_EQ(front_right_velocity1, front_right_velocity2);
//}

//TEST_F(RobotAutoTestFixture, TestFrontLeftMotorVelocity) {
//
//    WheelSpace_t prev_wheel_velocities = {0.0, 0.0, 0.0, 0.0};
//    WheelSpace_t target_wheel_velocities = {0.0, 0.0, 0.0, 0.0};
//    EuclideanSpace_t target_linear_velocity  = {0.5, 0.5, 0.0};
//
//    for (int i = 0; i < 10; i++) {
//        target_wheel_velocities = motor_service_->rampWheelVelocity(prev_wheel_velocities, target_linear_velocity, static_cast<double>(robot_constants_.robot_max_speed_m_per_s),
//                                                                    static_cast<double>(robot_constants_.robot_max_acceleration_m_per_s_2), 0.1);
//        prev_wheel_velocities = target_wheel_velocities;
//
//        // Set target speeds accounting for acceleration
//        motor_service_->writeIntToTMC4671(FRONT_LEFT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_TARGET,
//                                          static_cast<int>(target_wheel_velocities[FRONT_LEFT_WHEEL_SPACE_INDEX] *
//                                                           MotorService::ELECTRICAL_RPM_PER_MECHANICAL_MPS));
//        // Read the Motor Status value from the motor status
//        double front_left_velocity =
//                static_cast<double>(motor_service_->readVelocityFromTMC4671(FRONT_LEFT_MOTOR_CHIP_SELECT)) *
//                MotorService::MECHANICAL_MPS_PER_ELECTRICAL_RPM;
//
//        EXPECT_DOUBLE_EQ(front_left_velocity, target_wheel_velocities[FRONT_LEFT_WHEEL_SPACE_INDEX]);
//    }
//
//    // reset the motor velocity
//    motor_service_->writeIntToTMC4671(FRONT_LEFT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_TARGET, 0);
//}
//
//TEST_F(RobotAutoTestFixture, TestBackLeftMotorVelocity) {
//
//    WheelSpace_t prev_wheel_velocities = {0.0, 0.0, 0.0, 0.0};
//    WheelSpace_t target_wheel_velocities = {0.0, 0.0, 0.0, 0.0};
//    EuclideanSpace_t target_linear_velocity  = {0.5, 0.5, 0.0};
//
//    for (int i = 0; i < 10; i++) {
//        target_wheel_velocities = motor_service_->rampWheelVelocity(prev_wheel_velocities, target_linear_velocity, static_cast<double>(robot_constants_.robot_max_speed_m_per_s),
//                                                                    static_cast<double>(robot_constants_.robot_max_acceleration_m_per_s_2), 0.1);
//        prev_wheel_velocities = target_wheel_velocities;
//
//        // Set target speeds accounting for acceleration
//        motor_service_->writeIntToTMC4671(BACK_LEFT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_TARGET,
//                                          static_cast<int>(target_wheel_velocities[BACK_LEFT_WHEEL_SPACE_INDEX] *
//                                                           MotorService::ELECTRICAL_RPM_PER_MECHANICAL_MPS));
//        // Read the Motor Status value from the motor status
//        double back_left_velocity =
//                static_cast<double>(motor_service_->readVelocityFromTMC4671(BACK_LEFT_MOTOR_CHIP_SELECT)) *
//                MotorService::MECHANICAL_MPS_PER_ELECTRICAL_RPM;
//
//        EXPECT_DOUBLE_EQ(back_left_velocity, target_wheel_velocities[BACK_LEFT_WHEEL_SPACE_INDEX]);
//    }
//
//    // reset the motor velocity
//    motor_service_->writeIntToTMC4671(BACK_LEFT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_TARGET, 0);
//}
//
//TEST_F(RobotAutoTestFixture, TestBackRightMotorVelocity) {
//
//    WheelSpace_t prev_wheel_velocities = {0.0, 0.0, 0.0, 0.0};
//    WheelSpace_t target_wheel_velocities = {0.0, 0.0, 0.0, 0.0};
//    EuclideanSpace_t target_linear_velocity  = {0.5, 0.5, 0.0};
//
//    for (int i = 0; i < 10; i++) {
//        target_wheel_velocities = motor_service_->rampWheelVelocity(prev_wheel_velocities, target_linear_velocity, static_cast<double>(robot_constants_.robot_max_speed_m_per_s),
//                                                                    static_cast<double>(robot_constants_.robot_max_acceleration_m_per_s_2), 0.1);
//        prev_wheel_velocities = target_wheel_velocities;
//
//        // Set target speeds accounting for acceleration
//        motor_service_->writeIntToTMC4671(BACK_RIGHT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_TARGET,
//                                          static_cast<int>(target_wheel_velocities[BACK_RIGHT_WHEEL_SPACE_INDEX] *
//                                                           MotorService::ELECTRICAL_RPM_PER_MECHANICAL_MPS));
//        // Read the Motor Status value from the motor status
//        double back_left_velocity =
//                static_cast<double>(motor_service_->readVelocityFromTMC4671(BACK_RIGHT_MOTOR_CHIP_SELECT)) *
//                MotorService::MECHANICAL_MPS_PER_ELECTRICAL_RPM;
//
//        EXPECT_DOUBLE_EQ(back_left_velocity, target_wheel_velocities[BACK_RIGHT_WHEEL_SPACE_INDEX]);
//    }
//
//    // reset the motor velocity
//    motor_service_->writeIntToTMC4671(BACK_RIGHT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_TARGET, 0);
//}