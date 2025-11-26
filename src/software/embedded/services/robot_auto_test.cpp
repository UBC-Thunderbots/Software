#include <chrono>
#include <thread>

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/embedded/motor_controller/stspin_motor_controller.h"
#include "software/embedded/motor_controller/tmc_motor_controller.h"
#include "software/embedded/primitive_executor.h"
#include "software/embedded/services/motor.h"
#include "software/embedded/services/power.h"
#include "software/logger/network_logger.h"

std::unique_ptr<MotorController> motor_controller_;
std::unique_ptr<PowerService> power_service_;

std::string runtime_dir    = "/tmp/tbots/yellow_test";

int main(int argc, char **argv)
{
    LoggerSingleton::initializeLogger(runtime_dir, nullptr, false);

    LOG(INFO) << "Running Robot Auto Test";

    motor_controller_ = std::make_unique<StSpinMotorController>();

    motor_controller_->setup();

    LOG(INFO) << "Motor controller setup complete";

    for (int i = 0; i < 100000; ++i)
    {
        motor_controller_->readThenWriteVelocity(MotorIndex::FRONT_RIGHT, 500);
        motor_controller_->readThenWriteVelocity(MotorIndex::FRONT_LEFT, 300);
        motor_controller_->readThenWriteVelocity(MotorIndex::BACK_LEFT, 300);
        motor_controller_->readThenWriteVelocity(MotorIndex::BACK_RIGHT, 300);

        LOG(INFO) << "Spinning all motors";

        std::this_thread::sleep_for(std::chrono::seconds(3));

        motor_controller_->readThenWriteVelocity(MotorIndex::FRONT_RIGHT, 0);
        motor_controller_->readThenWriteVelocity(MotorIndex::FRONT_LEFT, 0);
        motor_controller_->readThenWriteVelocity(MotorIndex::BACK_LEFT, 0);
        motor_controller_->readThenWriteVelocity(MotorIndex::BACK_RIGHT, 0);

        motor_controller_->checkDriverFault(MotorIndex::FRONT_RIGHT);
        motor_controller_->checkDriverFault(MotorIndex::FRONT_LEFT);
        motor_controller_->checkDriverFault(MotorIndex::BACK_LEFT);
        motor_controller_->checkDriverFault(MotorIndex::BACK_RIGHT);

        LOG(INFO) << "Completed iteration " << i;

        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    LOG(INFO) << "Robot Auto Test Complete";

    return 0;
}
