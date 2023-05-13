#include <gtest/gtest.h>

#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/services/motor.h"
#include "software/jetson_nano/services/power.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"
#include "software/jetson_nano/services/network/power_service_exception.h"
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
//    void SetUp(void) {
//        motor_service_->setUpMotors();
//
//    }
//
//    void TearDown(void) {
//        // reset the motor velocity
//        for (uint8_t motor = 0; motor < NUM_DRIVE_MOTORS; motor++)
//        {
//            motor_service_->setTargetVelocity(motor, 0);
//        }
//    }

    RobotAutoTestFixture()
    {
//        motor_service_ = std::make_unique<MotorService>(
//            create2021RobotConstants(), CONTROL_LOOP_HZ);
        //power_service_ = std::make_unique<PowerService>();
//        robot_constants_ = create2021RobotConstants();
    }

    ~RobotAutoTestFixture() {}

    // Services
    std::unique_ptr<MotorService>
        motor_service_;  // TODO: loop_hz is not being used in motor service
    std::unique_ptr<PowerService> power_service_;
    RobotConstants_t robot_constants_;

    // SPI Chip Selects
    static const uint8_t FRONT_LEFT_MOTOR_CHIP_SELECT  = 0;
    static const uint8_t FRONT_RIGHT_MOTOR_CHIP_SELECT = 3;
    static const uint8_t BACK_LEFT_MOTOR_CHIP_SELECT   = 1;
    static const uint8_t BACK_RIGHT_MOTOR_CHIP_SELECT  = 2;
    static const uint8_t NUM_DRIVE_MOTORS              = 4;
};

//TEST_F(RobotAutoTestFixture, SPITransferFrontRightMotorTest) {
//
//    // Check driver fault
//    if (motor_service_->checkDriverFault(FRONT_RIGHT_MOTOR_CHIP_SELECT)) {
//        LOG(FATAL) << "Detected Motor Fault";
//    }
//
//    // We do not need to set up or calibrate the motors
//    motor_service_->writeIntToTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_ADDR, 0x000000000);
//    int read_value = motor_service_->readIntFromTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_DATA);
//
//    // Check if CHIPINFO_DATA returns 0x34363731
//    if (read_value == 875968305) {
//        LOG(INFO) << "SPI Transfer is successful";
//    } else {
//        LOG(FATAL) << "SPI Transfer not successful";
//    }
//}
//
//TEST_F(RobotAutoTestFixture, SPITransferFrontLeftMotorTest) {
//
//    // Check driver fault
//    if (motor_service_->checkDriverFault(FRONT_LEFT_MOTOR_CHIP_SELECT)) {
//        LOG(FATAL) << "Detected Motor Fault";
//    }
//
//    // We do not need to set up or calibrate the motors
//    motor_service_->writeIntToTMC4671(FRONT_LEFT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_ADDR, 0x000000000);
//    int read_value = motor_service_->readIntFromTMC4671(FRONT_LEFT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_DATA);
//
//    // Check if CHIPINFO_DATA returns 0x34363731
//    if (read_value == 875968305) {
//        LOG(INFO) << "SPI Transfer is successful";
//    } else {
//        LOG(FATAL) << "SPI Transfer not successful";
//    }
//
//}(
//
//TEST_F(RobotAutoTestFixture, SPITransferBackLeftMotorTest) {
//
//    // Check driver fault
//    if (motor_service_->checkDriverFault(BACK_LEFT_MOTOR_CHIP_SELECT)) {
//        LOG(FATAL) << "Detected Motor Fault";
//    }
//
//    // We do not need to set up or calibrate the motors
//    motor_service_->writeIntToTMC4671(BACK_LEFT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_ADDR, 0x000000000);
//    int read_value = motor_service_->readIntFromTMC4671(BACK_LEFT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_DATA);
//
//    // Check if CHIPINFO_DATA returns 0x34363731
//    if (read_value == 875968305) {
//        LOG(INFO) << "SPI Transfer is successful";
//    } else {
//        LOG(FATAL) << "SPI Transfer not successful";
//    }
//
//}
//
//TEST_F(RobotAutoTestFixture, SPITransferBackRightMotorTest) {
//
//    // Check driver fault
//    if (motor_service_->checkDriverFault(BACK_RIGHT_MOTOR_CHIP_SELECT)) {
//        LOG(FATAL) << "Detected Motor Fault";
//    }
//
//    // We do not need to set up or calibrate the motors
//    motor_service_->writeIntToTMC4671(BACK_RIGHT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_ADDR, 0x000000000);
//    int read_value = motor_service_->readIntFromTMC4671(BACK_RIGHT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_DATA);
//
//    // Check if CHIPINFO_DATA returns 0x34363731
//    if (read_value == 875968305) {
//        LOG(INFO) << "SPI Transfer is successful";
//    } else {
//        LOG(FATAL) << "SPI Transfer not successful";
//    }
//
//}

TEST_F(RobotAutoTestFixture, PowerboardConnectionTest) {

    EXPECT_NO_THROW({
                        PowerService();
                    }) << "Unable to communicate with the power board";

//    try {
//        PowerService();
//    } catch (const PowerServiceException &pse) {
//        EXPECT_EQ(pse.what(), std::string("USB not plugged into Jetson Nano"));
//        LOG(INFO) << "Connection exception caught: " << pse.what() << std::endl;
//    }

}

//TEST_F(RobotAutoTestFixture, TestFrontRightMotorVelocity) {
//
//    // 1. Check driver fault, make the function public
//    if (motor_service_->checkDriverFault(FRONT_RIGHT_MOTOR_CHIP_SELECT)) {
//        LOG(FATAL) << "Detected Motor Fault";
//    }
//
//    // 3. Abstract out drive motor setup into a function
//    motor_service_->setUpDriveMotor(FRONT_RIGHT_MOTOR_CHIP_SELECT);
//
//    // 2. Log messages for motor calibration
//    motor_service_->startEncoderCalibration(FRONT_RIGHT_MOTOR_CHIP_SELECT);
//    LOG(INFO) << "Starting motor encoder calibration";
//    motor_service_->endEncoderCalibration(FRONT_RIGHT_MOTOR_CHIP_SELECT);
//    LOG(INFO) << "Ending motor encoder calibration";
//
//    WheelSpace_t prev_wheel_velocities = {0.0, 0.0, 0.0, 0.0};
//    WheelSpace_t target_wheel_velocities = {0.0, 0.0, 0.0, 0.0};
//    EuclideanSpace_t target_linear_velocity  = {0.1, 0.1, 0.0};
//
//    for (int i = 0; i < 10; i++) {
//        target_wheel_velocities = motor_service_->rampWheelVelocity(prev_wheel_velocities, target_linear_velocity, static_cast<double>(robot_constants_.robot_max_speed_m_per_s),
//                                                                    static_cast<double>(robot_constants_.robot_max_acceleration_m_per_s_2), 0.01);
//        prev_wheel_velocities = target_wheel_velocities;
//
//        // Set target speeds accounting for acceleration
//        int write_value = static_cast<int>(target_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX] * MotorService::ELECTRICAL_RPM_PER_MECHANICAL_MPS);
//        motor_service_->writeIntToTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_TARGET, write_value);
//        int read_value = motor_service_->readIntFromTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_TARGET);
//
//
//        if (write_value != read_value) {
//            FAIL() << "Read and write value are different";
//        } else {
//            LOG(INFO) << "Read and write value are same";
//        }
//
//        for (int i = 0; i < 10; i++) {
//            int read_value_target = motor_service_->readIntFromTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_TARGET);
//            int read_value_actual = motor_service_->readIntFromTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_ACTUAL);
//
//            LOG(INFO) << "Target Velocity " << read_value_target;
//            LOG(INFO) << "Actual Velocity " << read_value_actual;
//
//            std::this_thread::sleep_for(std::chrono::milliseconds(10));
//
//        }
//
//        // Read the Motor Status value from the motor status
//        double front_right_velocity =
//                static_cast<double>(motor_service_->readVelocityFromTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT));
//        LOG(INFO) << target_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX] << "Target wheel velocity";
//        LOG(INFO) << front_right_velocity << "Actual wheel velocity read";
//
//        // Run-away protection
//        motor_service_->disableVelocity(front_right_velocity, prev_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX]);
//
//
//        // 4. Make the threshold more lenient for expect equal double
//        //EXPECT_DOUBLE_EQ(front_right_velocity, target_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX]);
//        ASSERT_NEAR(front_right_velocity, target_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX], 0.01);
//
//        // 5. Check driver faults again after making updates to motor
//        motor_service_->checkDriverFault(FRONT_RIGHT_MOTOR_CHIP_SELECT);
//
//        // 6. Detect the reset than
//        int reset_detector = motor_service_->readIntFromTMC4671(0, TMC4671_PID_ACCELERATION_LIMIT);
//
//        // When the motor board is reset the value in the above register is set to the maximum
//        // signed 32 bit value Please read the header file and the datasheet for more info
//        if (reset_detector == 2147483647)
//        {
//            motor_service_->checkDriverFault(FRONT_RIGHT_MOTOR_CHIP_SELECT);
//            motor_service_->setUpMotors();
//            motor_service_->startEncoderCalibration(FRONT_RIGHT_MOTOR_CHIP_SELECT);
//            motor_service_->endEncoderCalibration(FRONT_RIGHT_MOTOR_CHIP_SELECT);
//            FAIL() << "RESET DETECTED";
//        }
//    }
//
//    // reset the motor velocity
//    for (uint8_t motor = 0; motor < NUM_DRIVE_MOTORS; motor++)
//    {
//        motor_service_->setTargetVelocity(motor, 0);
//    }
//
//}
