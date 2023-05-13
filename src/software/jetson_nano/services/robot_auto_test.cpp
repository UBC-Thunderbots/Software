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

TEST_F(RobotAutoTestFixture, SPITransferFrontRightMotorTest) {

    motor_service_ = std::make_unique<MotorService>(
            create2021RobotConstants(), CONTROL_LOOP_HZ);

    // Check driver fault
    if (motor_service_->checkDriverFault(FRONT_RIGHT_MOTOR_CHIP_SELECT)) {
        LOG(FATAL) << "Detected Motor Fault";
    }

    // We do not need to set up or calibrate the motors
    motor_service_->writeIntToTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_ADDR, 0x000000000);
    int read_value = motor_service_->readIntFromTMC4671(FRONT_RIGHT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_DATA);

    // Check if CHIPINFO_DATA returns 0x34363731
    if (read_value == 875968305) {
        LOG(INFO) << "SPI Transfer is successful";
    } else {
        LOG(FATAL) << "SPI Transfer not successful";
    }
}

TEST_F(RobotAutoTestFixture, SPITransferFrontLeftMotorTest) {

    motor_service_ = std::make_unique<MotorService>(
            create2021RobotConstants(), CONTROL_LOOP_HZ);

    // Check driver fault
    if (motor_service_->checkDriverFault(FRONT_LEFT_MOTOR_CHIP_SELECT)) {
        LOG(FATAL) << "Detected Motor Fault";
    }

    // We do not need to set up or calibrate the motors
    motor_service_->writeIntToTMC4671(FRONT_LEFT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_ADDR, 0x000000000);
    int read_value = motor_service_->readIntFromTMC4671(FRONT_LEFT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_DATA);

    // Check if CHIPINFO_DATA returns 0x34363731
    if (read_value == 875968305) {
        LOG(INFO) << "SPI Transfer is successful";
    } else {
        LOG(FATAL) << "SPI Transfer not successful";
    }

}

TEST_F(RobotAutoTestFixture, SPITransferBackLeftMotorTest) {

    motor_service_ = std::make_unique<MotorService>(
            create2021RobotConstants(), CONTROL_LOOP_HZ);

    // Check driver fault
    if (motor_service_->checkDriverFault(BACK_LEFT_MOTOR_CHIP_SELECT)) {
        LOG(FATAL) << "Detected Motor Fault";
    }

    // We do not need to set up or calibrate the motors
    motor_service_->writeIntToTMC4671(BACK_LEFT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_ADDR, 0x000000000);
    int read_value = motor_service_->readIntFromTMC4671(BACK_LEFT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_DATA);

    // Check if CHIPINFO_DATA returns 0x34363731
    if (read_value == 875968305) {
        LOG(INFO) << "SPI Transfer is successful";
    } else {
        LOG(FATAL) << "SPI Transfer not successful";
    }

}

TEST_F(RobotAutoTestFixture, SPITransferBackRightMotorTest) {

    motor_service_ = std::make_unique<MotorService>(
            create2021RobotConstants(), CONTROL_LOOP_HZ);

    // Check driver fault
    if (motor_service_->checkDriverFault(BACK_RIGHT_MOTOR_CHIP_SELECT)) {
        LOG(FATAL) << "Detected Motor Fault";
    }

    // We do not need to set up or calibrate the motors
    motor_service_->writeIntToTMC4671(BACK_RIGHT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_ADDR, 0x000000000);
    int read_value = motor_service_->readIntFromTMC4671(BACK_RIGHT_MOTOR_CHIP_SELECT, TMC4671_CHIPINFO_DATA);

    // Check if CHIPINFO_DATA returns 0x34363731
    if (read_value == 875968305) {
        LOG(INFO) << "SPI Transfer is successful";
    } else {
        LOG(FATAL) << "SPI Transfer not successful";
    }

}

TEST_F(RobotAutoTestFixture, PowerboardConnectionTest) {

    EXPECT_NO_THROW({
                        PowerService();
                    }) << "Unable to communicate with the power board";

}
