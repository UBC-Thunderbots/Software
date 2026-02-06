#include <chrono>
#include <thread>

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "software/embedded/motor_controller/stspin_motor_controller.h"
#include "software/embedded/primitive_executor.h"
#include "software/embedded/services/motor.h"
#include "software/embedded/services/power.h"
#include "software/logger/network_logger.h"

std::unique_ptr<StSpinMotorController> motor_controller_;
std::unique_ptr<PowerService> power_service_;

std::string runtime_dir = "/tmp/tbots/yellow_test";

void runRobotAutoTest()
{
    LOG(INFO) << "Running Robot Auto Test";

    motor_controller_ = std::make_unique<StSpinMotorController>();
    motor_controller_->setup();

    for (auto motor : driveMotors())
    {
        motor_controller_->sendAndReceiveFrame(
            motor, SetPidTorqueKpKiFrame{.kp = 1250, .ki = 2000});
        motor_controller_->sendAndReceiveFrame(
            motor, SetPidFluxKpKiFrame{.kp = 1250, .ki = 2000});
        motor_controller_->sendAndReceiveFrame(
            motor, SetPidSpeedKpKiFrame{.kp = 128, .ki = 100});
    }

    LOG(INFO) << "Motor controller setup complete";

    LOG(INFO) << "Running torque step test";

    for (auto motor : driveMotors())
    {
        motor_controller_->sendAndReceiveFrame(
            motor, SetResponseTypeFrame{StSpinResponseType::IQ_AND_ID});
    }

    for (const int16_t torque : {0, 250, 500, 250, 0})
    {
        for (int i = 0; i < 1000; ++i)
        {
            for (auto motor : driveMotors())
            {
                motor_controller_->sendAndReceiveFrame(motor,
                                                       SetTargetTorqueFrame{
                                                           .motor_enabled       = true,
                                                           .motor_target_torque = torque,
                                                       });

                LOG(PLOTJUGGLER) << *createPlotJugglerValue({
                    {"Iq", motor_controller_->motor_iq_.at(motor)},
                    {"Id", motor_controller_->motor_id_.at(motor)},
                });
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    LOG(INFO) << "Running speed step test";

    for (auto motor : driveMotors())
    {
        motor_controller_->sendAndReceiveFrame(
            motor, SetResponseTypeFrame{StSpinResponseType::SPEED_AND_FAULTS});
    }

    for (const int16_t speed : {0, 250, 500, 250, 0})
    {
        for (int i = 0; i < 1000; ++i)
        {
            for (auto motor : driveMotors())
            {
                motor_controller_->readThenWriteVelocity(motor, speed);

                LOG(PLOTJUGGLER) << *createPlotJugglerValue({
                    {"target_speed_rpm", speed},
                    {"measured_speed_rpm",
                     motor_controller_->motor_measured_speed_rpm_.at(motor)},
                });
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    motor_controller_->immediatelyDisable();

    LOG(INFO) << "Robot Auto Test Complete";
}

int main(int argc, char **argv)
{
    LoggerSingleton::initializeLogger(runtime_dir, nullptr, false);
    runRobotAutoTest();
    return 0;
}
