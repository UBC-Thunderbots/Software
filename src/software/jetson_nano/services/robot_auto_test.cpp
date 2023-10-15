#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/services/motor.h"
#include "software/jetson_nano/services/power.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/message_translation/tbots_geometry.h"
#include "software/logger/network_logger.h"

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

static const uint8_t CHIP_SELECT[] = {motor_service_->FRONT_LEFT_MOTOR_CHIP_SELECT, motor_service_->FRONT_RIGHT_MOTOR_CHIP_SELECT, motor_service_->BACK_LEFT_MOTOR_CHIP_SELECT, motor_service_->BACK_RIGHT_MOTOR_CHIP_SELECT};

constexpr int ASCII_4671_IN_HEXADECIMAL = 0x34363731;
constexpr double THRESHOLD = 0.0001;
constexpr int DELAY_NS = 10000;
std::string runtime_dir  = "/tmp/tbots/yellow_test";

int main(int argc, char **argv) {
    LoggerSingleton::initializeLogger(runtime_dir);
    LOG(INFO) << "Running on the Jetson Nano!";

    motor_service_ =
        std::make_unique<MotorService>(create2021RobotConstants(), CONTROL_LOOP_HZ);

    // Testing Motor board SPI transfer
    for (uint8_t chip_select : CHIP_SELECT)
    {
        // Check driver fault
        if (!motor_service_->checkDriverFault(chip_select).drive_enabled)
        {
            LOG(FATAL) << "Detected Motor Fault";
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
            LOG(FATAL) << "SPI Transfer not successful";
        }
    }

    // Testing Power board SPI transfer
    try
    {
        power_service_ = std::make_unique<PowerService>();

        usleep(DELAY_NS);

        TbotsProto::PowerStatus power_status =
            power_service_->poll(TbotsProto::PowerControl(), 0, 0, 0);
        power_status = power_service_->poll(TbotsProto::PowerControl(), 0, 0, 0);

        if (abs(power_status.battery_voltage() - 0) < THRESHOLD)
        {
            LOG(FATAL) << "Battery voltage is zero";
        }
        else if (abs(power_status.capacitor_voltage() - 0) < THRESHOLD)
        {
            LOG(FATAL) << "Capacitor voltage is zero";
        }
        else if (abs(power_status.current_draw() - 0) < THRESHOLD)
        {
            LOG(FATAL) << "Current draw is zero";
        }
        else if (power_status.sequence_num() == 0)
        {
            LOG(FATAL) << "Sequence number is zero";
        }
    }
    catch (std::runtime_error &e)
    {
        LOG(FATAL) << "Unable to communicate with the power board";
    }
}
