#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/embedded/primitive_executor.h"
#include "software/embedded/services/motor.h"
#include "software/embedded/services/power.h"
#include "software/logger/network_logger.h"
#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"

extern "C"
{
#include "external/trinamic/tmc/ic/TMC4671/TMC4671.h"
#include "external/trinamic/tmc/ic/TMC4671/TMC4671_Variants.h"
#include "external/trinamic/tmc/ic/TMC6100/TMC6100.h"
}

std::unique_ptr<MotorService> motor_service_;
std::unique_ptr<PowerService> power_service_;
RobotConstants_t robot_constants_;
int read_value;

// SPI Chip Selects

static const uint8_t CHIP_SELECT[] = {motor_service_->FRONT_LEFT_MOTOR_CHIP_SELECT,
                                      motor_service_->FRONT_RIGHT_MOTOR_CHIP_SELECT,
                                      motor_service_->BACK_LEFT_MOTOR_CHIP_SELECT,
                                      motor_service_->BACK_RIGHT_MOTOR_CHIP_SELECT, 4};

constexpr int ASCII_4671_IN_HEXADECIMAL = 0x34363731;
constexpr double THRESHOLD              = 0.0001;
constexpr int DELAY_NS                  = 10000;
std::string runtime_dir                 = "/tmp/tbots/yellow_test";

int main(int argc, char **argv)
{
    LoggerSingleton::initializeLogger(runtime_dir, nullptr, false);

    LOG(INFO) << "Running on the Jetson Nano!";

    motor_service_ =
        std::make_unique<MotorService>(create2021RobotConstants(), THUNDERLOOP_HZ);

    // Testing Motor board SPI transfer
    for (uint8_t chip_select : CHIP_SELECT)
    {
        LOG(INFO) << "Checking motor: " << int(chip_select);

        // Check driver fault
        if (!motor_service_->checkDriverFault(chip_select).drive_enabled)
        {
            LOG(WARNING) << "Detected Motor Fault";
        }

        motor_service_->writeIntToTMC4671(chip_select, TMC4671_CHIPINFO_ADDR,
                                          0x000000000);
        read_value =
            motor_service_->readIntFromTMC4671(chip_select, TMC4671_CHIPINFO_DATA);

        // Check if CHIPINFO_DATA returns 0x34363731
        if (read_value == ASCII_4671_IN_HEXADECIMAL)
        {
            LOG(INFO) << "SPI Transfer is successful";
        }
        else
        {
            LOG(WARNING) << "SPI Transfer not successful";
        }
    }

    motor_service_->resetMotorBoard();

    // Testing Power board UART transfer
    try
    {
        power_service_ = std::make_unique<PowerService>();

        usleep(DELAY_NS);

        TbotsProto::PowerStatus power_status =
            power_service_->poll(TbotsProto::PowerControl(), 0, 0, 0, 0);
        power_status = power_service_->poll(TbotsProto::PowerControl(), 0, 0, 0, 0);

        if (abs(power_status.battery_voltage()) < THRESHOLD)
        {
            LOG(WARNING) << "Battery voltage is zero";
        }
        else if (abs(power_status.capacitor_voltage()) < THRESHOLD)
        {
            LOG(WARNING) << "Capacitor voltage is zero";
        }
        else if (abs(power_status.current_draw()) < THRESHOLD)
        {
            LOG(WARNING) << "Current draw is zero";
        }
        else if (power_status.sequence_num() == 0)
        {
            LOG(WARNING) << "Sequence number is zero";
        }
    }
    catch (std::runtime_error &e)
    {
        LOG(WARNING) << "Unable to communicate with the power board";
    }

    return 0;
}
