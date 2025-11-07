#include <chrono>
#include <thread>

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/embedded/motor_controller/motor_board.h"
#include "software/embedded/motor_controller/stspin_motor_controller.h"
#include "software/embedded/motor_controller/tmc_motor_controller.h"
#include "software/embedded/primitive_executor.h"
#include "software/embedded/services/motor.h"
#include "software/embedded/services/power.h"
#include "software/logger/network_logger.h"
#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"

std::unique_ptr<StSpinMotorController> motor_controller_;
std::unique_ptr<PowerService> power_service_;
RobotConstants_t robot_constants_;
int read_value;

// SPI Chip Selects
constexpr double THRESHOLD = 0.0001;
constexpr int DELAY_NS     = 10000;
std::string runtime_dir    = "/tmp/tbots/yellow_test";

int main(int argc, char **argv)
{
    LoggerSingleton::initializeLogger(runtime_dir, nullptr, false);

    LOG(INFO) << "Running Robot Auto Test";

    motor_controller_ = std::make_unique<StSpinMotorController>();

    motor_controller_->setup();

    LOG(INFO) << "Motor controller setup complete";

    LOG(INFO) << "Waiting...";
    std::this_thread::sleep_for(std::chrono::seconds(1));

    motor_controller_->readThenWriteVelocity(MotorIndex::BACK_RIGHT, 1500);
    LOG(INFO) << "Waiting...";
    std::this_thread::sleep_for(std::chrono::seconds(2));
    motor_controller_->readThenWriteVelocity(MotorIndex::BACK_RIGHT, 0);



    /*if (motor_controller_->earlyPoll() == MotorControllerStatus::OK)
    {
        for (int i = 0; i <= 100; ++i)
        {
            motor_controller_->readThenWriteVelocity(MotorIndex::FRONT_RIGHT, 1500);
            LOG(INFO) << "Waiting...";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        motor_controller_->readThenWriteVelocity(MotorIndex::FRONT_RIGHT, 0);
    }*/


    /*
        for (int i = 0; i < 1000; ++i)
        {
            motor_controller_->sendAndReceiveFrame(MotorIndex::BACK_RIGHT,
       StSpinOpcode::SPI_NOOP);
        }
    */

    //	motor_controller_->reset();

    /*
        motor_controller_->sendAndReceiveFrame(MotorIndex::FRONT_LEFT,
       StSpinOpcode::MOV_AX, static_cast<int16_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count() % 5000
            ));
    */

    /*
        motor_controller_->sendAndReceiveFrame(MotorIndex::FRONT_LEFT,
       StSpinOpcode::MOV_BX, 1000);
        motor_controller_->sendAndReceiveFrame(MotorIndex::FRONT_LEFT,
       StSpinOpcode::SET_SPEEDRAMP);
        motor_controller_->sendAndReceiveFrame(MotorIndex::FRONT_LEFT,
       StSpinOpcode::START_MOTOR);
    */
    LOG(INFO) << "Robot Auto Test Complete";

    return 0;
}
