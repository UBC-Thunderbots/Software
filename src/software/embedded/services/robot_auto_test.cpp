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

#include <chrono>
#include <thread>

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

  	if (motor_controller_->earlyPoll() == MotorControllerStatus::OK)
    {
  		motor_controller_->readThenWriteVelocity(MotorIndex::FRONT_LEFT, 60);
      	std::this_thread::sleep_for(std::chrono::seconds(15));
  		motor_controller_->readThenWriteVelocity(MotorIndex::FRONT_LEFT, 0);
  	}

//    for (int i = 0; i < 1000; ++i)
//    {
//        motor_controller_->sendAndReceiveFrame(MotorIndex::FRONT_LEFT, StSpinOpcode::SPI_NOOP);
//    }

	motor_controller_->reset();

    LOG(INFO) << "Robot Auto Test Complete";

    return 0;
}
