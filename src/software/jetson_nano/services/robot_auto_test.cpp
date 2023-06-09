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

std::unique_ptr<MotorService>
        motor_service_;  // TODO: loop_hz is not being used in motor service
std::unique_ptr<PowerService> power_service_;
RobotConstants_t robot_constants_;
int read_value;

// SPI Chip Selects
static const uint8_t FRONT_LEFT_MOTOR_CHIP_SELECT  = 0;
static const uint8_t FRONT_RIGHT_MOTOR_CHIP_SELECT = 3;
static const uint8_t BACK_LEFT_MOTOR_CHIP_SELECT   = 1;
static const uint8_t BACK_RIGHT_MOTOR_CHIP_SELECT  = 2;

static const uint8_t CHIP_SELECT[] = {FRONT_LEFT_MOTOR_CHIP_SELECT, FRONT_RIGHT_MOTOR_CHIP_SELECT, BACK_LEFT_MOTOR_CHIP_SELECT, BACK_RIGHT_MOTOR_CHIP_SELECT};

int main(int argc, char **argv) {
    std::cout << "running on the jetson nano!" << std::endl;
    std::cout << "running ansible re-build robot_auto_test_binary" << std::endl;

    // Testing Motor board SPI transfer
    for (uint8_t chip_select : CHIP_SELECT) {
        motor_service_ = std::make_unique<MotorService>(
                create2021RobotConstants(), CONTROL_LOOP_HZ);

        // Check driver fault
        if (motor_service_->checkDriverFault(chip_select)) {
            LOG(FATAL) << "Detected Motor Fault";
        }

        motor_service_->writeIntToTMC4671(chip_select, TMC4671_CHIPINFO_ADDR, 0x000000000);
        read_value = motor_service_->readIntFromTMC4671(chip_select, TMC4671_CHIPINFO_DATA);

        // Check if CHIPINFO_DATA returns 0x34363731
        if (read_value == 875968305) {
            LOG(INFO) << "SPI Transfer is successful";
        } else {
            LOG(FATAL) << "SPI Transfer not successful";
        }
    }

    // Testing Power board SPI transfer
    try {
        PowerService();
    } catch (const PowerServiceException &pse) {
        LOG(FATAL) << "Unable to communicate with the power board";
    }
}
