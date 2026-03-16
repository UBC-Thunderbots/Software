#include "software/embedded/motor_controller/stspin_motor_controller.h"

#include <boost/program_options.hpp>
#include <chrono>
#include <csignal>
#include <thread>
#include <utility>

#include "software/logger/logger.h"
#include "software/logger/network_logger.h"

// Flag to indicate when a stop has been requested (e.g. via Ctrl+C)
static volatile std::sig_atomic_t g_stop_requested = false;

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
        int setpoint_duration_ms = 2000;
        std::vector<int16_t> setpoints;
        std::unordered_set<MotorIndex> enabled_motors;
        PidOption torque_pid;
        PidOption flux_pid;
        PidOption speed_pid;
    };

    explicit StSpinMotorControllerTest(CommandLineArgs args) : args_(std::move(args)) {}

    void runMotorTest() const
    {
        LOG(INFO) << "Running Motor Test";

        std::stringstream ss;
        for (const MotorIndex motor : args_.enabled_motors)
        {
            ss << motor << " ";
        }
        LOG(INFO) << "Enabled motors: " << ss.str();

        const auto motor_controller = std::make_unique<StSpinMotorController>();
        motor_controller->setup();

        LOG(INFO) << "Motor controller setup complete";

        for (const MotorIndex motor : args_.enabled_motors)
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

        LOG(INFO) << "Running " << args_.mode << " profile";

        for (const int16_t setpoint : args_.setpoints)
        {
            if (g_stop_requested)
            {
                break;
            }

            StSpinMotorController::OutgoingFrame command_frame;
            if (args_.mode == "speed")
            {
                command_frame = SetTargetSpeedFrame{
                    .motor_enabled          = true,
                    .motor_target_speed_rpm = setpoint,
                };
            }
            else if (args_.mode == "torque")
            {
                command_frame = SetTargetTorqueFrame{
                    .motor_enabled       = true,
                    .motor_target_torque = setpoint,
                };
            }

            auto start_time = std::chrono::system_clock::now();
            auto duration = std::chrono::milliseconds(args_.setpoint_duration_ms);
            while (std::chrono::system_clock::now() - start_time < duration)
            {
                if (g_stop_requested)
                {
                    break;
                }

                for (const MotorIndex motor : args_.enabled_motors)
                {
                    motor_controller->sendAndReceiveFrame(motor, command_frame);
                }

                for (const MotorIndex motor : args_.enabled_motors)
                {
                    motor_controller->sendMotorStatusToPlotJuggler(motor);
                    motor_controller->checkDriverFault(motor);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }

        motor_controller->immediatelyDisable();

        LOG(INFO) << "Motor Test Complete";
    }

   private:
    CommandLineArgs args_;
};

static std::vector<int16_t> parseSetpoints(const std::string& setpoints_str)
{
    std::vector<int16_t> setpoints;
    std::stringstream ss(setpoints_str);
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

static std::unordered_set<MotorIndex> parseEnabledMotors(
    const std::string& enabled_motors_str)
{
    if (enabled_motors_str == "all")
    {
        constexpr auto all_motors = driveMotors();
        return {all_motors.begin(), all_motors.end()};
    }

    std::unordered_set<MotorIndex> enabled_motors;
    std::stringstream ss(enabled_motors_str);
    std::string item;

    while (std::getline(ss, item, ','))
    {
        if (item == "fl")
        {
            enabled_motors.insert(MotorIndex::FRONT_LEFT);
        }
        else if (item == "fr")
        {
            enabled_motors.insert(MotorIndex::FRONT_RIGHT);
        }
        else if (item == "bl")
        {
            enabled_motors.insert(MotorIndex::BACK_LEFT);
        }
        else if (item == "br")
        {
            enabled_motors.insert(MotorIndex::BACK_RIGHT);
        }
    }

    return enabled_motors;
}

int main(int argc, char** argv)
{
    NetworkLoggerSingleton::initializeLogger(0, true);

    StSpinMotorControllerTest::CommandLineArgs args;

    boost::program_options::options_description desc{"Options"};
    // clang-format off
    desc.add_options()
        ("help,h", boost::program_options::bool_switch(&args.help), "Show help")
        ("mode", boost::program_options::value<std::string>(&args.mode),
            "Control mode: speed | torque")
        ("setpoints", boost::program_options::value<std::string>(),
            "Comma-separated setpoints (required)")
        ("setpoint_duration", boost::program_options::value<int>(&args.setpoint_duration_ms),
            "Duration to hold each setpoint in milliseconds")
        ("enabled_motors", boost::program_options::value<std::string>(),
            "Motors to enable: 'all' (default) or comma-separated list (e.g. 'fl,fr,bl,br')")
        ("torque_kp", boost::program_options::value<int16_t>(), "Torque PID Kp")
        ("torque_ki", boost::program_options::value<int16_t>(), "Torque PID Ki")
        ("flux_kp", boost::program_options::value<int16_t>(), "Flux PID Kp")
        ("flux_ki", boost::program_options::value<int16_t>(), "Flux PID Ki")
        ("speed_kp", boost::program_options::value<int16_t>(), "Speed PID Kp")
        ("speed_ki", boost::program_options::value<int16_t>(), "Speed PID Ki");
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

    if (vm.count("enabled_motors"))
    {
        args.enabled_motors =
            parseEnabledMotors(vm.at("enabled_motors").as<std::string>());
    }
    else
    {
        args.enabled_motors = parseEnabledMotors("all");
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

    std::signal(SIGINT, [](int) { g_stop_requested = true; });

    StSpinMotorControllerTest stspin_motor_controller_test(args);
    stspin_motor_controller_test.runMotorTest();

    return 0;
}
