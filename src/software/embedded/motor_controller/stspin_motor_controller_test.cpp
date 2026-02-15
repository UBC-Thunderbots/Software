#include "software/embedded/motor_controller/stspin_motor_controller.h"

#include <boost/program_options.hpp>
#include <chrono>
#include <thread>

#include "proto/message_translation/tbots_protobuf.h"
#include "software/logger/logger.h"

class StSpinMotorControllerTest
{
   public:
    struct CommandLineArgs
    {
        struct PidOption
        {
            std::optional<int16_t> kp;
            std::optional<int16_t> ki;
        };

        bool help        = false;
        std::string mode = "speed";
        std::vector<int16_t> setpoints;
        PidOption torque_pid;
        PidOption flux_pid;
        PidOption speed_pid;
    };

    explicit StSpinMotorControllerTest(const CommandLineArgs& args) : args_(args) {}

    void runMotorTest() const
    {
        LOG(INFO) << "Running Motor Test";

        const auto motor_controller = std::make_unique<StSpinMotorController>();
        motor_controller->setup();

        LOG(INFO) << "Motor controller setup complete";

        const std::vector motors = {MotorIndex::FRONT_RIGHT};

        for (auto motor : motors)
        {
            if (args_.torque_pid.kp && args_.torque_pid.ki)
            {
                motor_controller->sendAndReceiveFrame(
                    motor, SetPidTorqueKpKiFrame{.kp = args_.torque_pid.kp.value(),
                                                 .ki = args_.torque_pid.ki.value()});
            }

            if (args_.flux_pid.kp && args_.flux_pid.ki)
            {
                motor_controller->sendAndReceiveFrame(
                    motor, SetPidFluxKpKiFrame{.kp = args_.flux_pid.kp.value(),
                                               .ki = args_.flux_pid.ki.value()});
            }

            if (args_.speed_pid.kp && args_.speed_pid.ki)
            {
                motor_controller->sendAndReceiveFrame(
                    motor, SetPidSpeedKpKiFrame{.kp = args_.speed_pid.kp.value(),
                                                .ki = args_.speed_pid.ki.value()});
            }
        }

        LOG(INFO) << "PID configuration complete";

        if (args_.mode == "speed")
        {
            LOG(INFO) << "Running speed profile";

            for (auto motor : motors)
            {
                motor_controller->sendAndReceiveFrame(
                    motor, SetResponseTypeFrame{StSpinResponseType::SPEED_AND_FAULTS});
            }

            for (const int16_t setpoint : args_.setpoints)
            {
                for (int i = 0; i < 2000; ++i)
                {
                    for (auto motor : motors)
                    {
                        motor_controller->readThenWriteVelocity(motor, setpoint);
                        motor_controller->checkDriverFault(motor);

                        LOG(PLOTJUGGLER) << *createPlotJugglerValue({
                            {"target_speed_rpm", setpoint},
                            {"measured_speed_rpm",
                             motor_controller->motor_status_.at(motor)
                                 .measured_speed_rpm},
                        });
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
            }
        }
        else if (args_.mode == "torque")
        {
            LOG(INFO) << "Running torque profile";

            for (auto motor : motors)
            {
                motor_controller->sendAndReceiveFrame(
                    motor, SetResponseTypeFrame{StSpinResponseType::IQ_AND_ID});
            }

            for (const int16_t setpoint : args_.setpoints)
            {
                for (int i = 0; i < 2000; ++i)
                {
                    for (auto motor : motors)
                    {
                        motor_controller->sendAndReceiveFrame(
                            motor, SetTargetTorqueFrame{
                                       .motor_enabled       = true,
                                       .motor_target_torque = setpoint,
                                   });

                        LOG(PLOTJUGGLER) << *createPlotJugglerValue({
                            {"Iq", motor_controller->motor_status_.at(motor).iq},
                            {"Id", motor_controller->motor_status_.at(motor).id},
                        });
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
            }
        }

        motor_controller->immediatelyDisable();

        LOG(INFO) << "Motor Test Complete";
    }

   private:
    CommandLineArgs args_;
};

static std::vector<int16_t> parseSetpoints(const std::string& setpointsString)
{
    std::vector<int16_t> setpoints;
    std::stringstream ss(setpointsString);
    std::string item;

    while (std::getline(ss, item, ','))
    {
        if (!item.empty())
        {
            setpoints.push_back(static_cast<int16_t>(std::stoi(item)));
        }
    }

    return setpoints;
}

int main(int argc, char** argv)
{
    std::string runtime_dir = "/tmp/tbots/motor_test";
    LoggerSingleton::initializeLogger(runtime_dir, nullptr);

    StSpinMotorControllerTest::CommandLineArgs args;

    boost::program_options::options_description desc{"Options"};
    // clang-format off
    desc.add_options()
        ("help,h", boost::program_options::bool_switch(&args.help), "Show help")
        ("mode", boost::program_options::value<std::string>(&args.mode),
            "Control mode: speed | torque")
        ("setpoints", boost::program_options::value<std::string>(),
            "Comma-separated setpoints (required)")
        ("torque_kp", boost::program_options::value<int16_t>(), "Torque PID kp")
        ("torque_ki", boost::program_options::value<int16_t>(), "Torque PID ki")
        ("flux_kp", boost::program_options::value<int16_t>(), "Flux PID kp")
        ("flux_ki", boost::program_options::value<int16_t>(), "Flux PID ki")
        ("speed_kp", boost::program_options::value<int16_t>(), "Speed PID kp")
        ("speed_ki", boost::program_options::value<int16_t>(), "Speed PID ki");
    // clang-format on

    boost::program_options::variables_map vm;
    boost::program_options::store(parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (args.help)
    {
        std::cout << desc << std::endl;
        return 0;
    }

    if (!vm.count("setpoints"))
    {
        std::cerr << "Error: setpoints not set" << std::endl;
        return 1;
    }

    args.setpoints = parseSetpoints(vm.at("setpoints").as<std::string>());

    if (vm.count("torque_kp"))
    {
        args.torque_pid.kp = vm.at("torque_kp").as<int16_t>();
    }

    if (vm.count("torque_ki"))
    {
        args.torque_pid.ki = vm.at("torque_ki").as<int16_t>();
    }

    if (vm.count("flux_kp"))
    {
        args.flux_pid.kp = vm.at("flux_kp").as<int16_t>();
    }

    if (vm.count("flux_ki"))
    {
        args.flux_pid.ki = vm.at("flux_ki").as<int16_t>();
    }

    if (vm.count("speed_kp"))
    {
        args.speed_pid.kp = vm.at("speed_kp").as<int16_t>();
    }

    if (vm.count("speed_ki"))
    {
        args.speed_pid.ki = vm.at("speed_ki").as<int16_t>();
    }

    StSpinMotorControllerTest stspin_motor_controller_test(args);
    stspin_motor_controller_test.runMotorTest();

    return 0;
}
