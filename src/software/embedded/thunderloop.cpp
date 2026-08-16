#include "software/embedded/thunderloop.h"

#include <Tracy.hpp>
#include <chrono>
#include <csignal>
#include <iostream>
#include <optional>
#include <thread>

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/robot_crash_msg.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/constants.h"
#include "software/constants.h"
#include "software/embedded/primitive_executor.h"
#include "software/embedded/services/imu.h"
#include "software/embedded/services/motor.h"
#include "software/logger/logger.h"
#include "software/logger/network_logger.h"
#include "software/tracy/tracy_constants.h"

// signal handling is done by csignal which requires a function pointer with C linkage
extern "C"
{
    static MotorService* g_motor_service         = NULL;
    static TbotsProto::RobotStatus* robot_status = NULL;
    static int channel_id;
    static std::string network_interface;
    static int robot_id;

    /**
     * Handles process signals
     *
     * @param the signal value (SIGINT, SIGABRT, SIGTERN, etc)
     */
    void tbotsExit(int signal_num)
    {
        if (g_motor_service)
        {
            g_motor_service->reset();
        }

        // by now g3log may have died due to the termination signal, so it isn't reliable
        // to log messages
        std::cerr << "\n\n!!!\nReceived termination signal: "
                  << g3::signalToStr(signal_num) << std::endl;
        std::cerr << "Thunderloop shutting down\n!!!\n" << std::endl;

        TbotsProto::RobotCrash crash_msg;
        auto dump = g3::internal::stackdump();
        crash_msg.set_robot_id(robot_id);
        crash_msg.set_stack_dump(dump);
        crash_msg.set_exit_signal(g3::signalToStr(signal_num));
        *(crash_msg.mutable_status()) = *robot_status;

        auto sender = ThreadedProtoUdpSender<TbotsProto::RobotCrash>(
            std::string(ROBOT_MULTICAST_CHANNELS.at(channel_id)), ROBOT_CRASH_PORT,
            network_interface, true);
        sender.sendProto(crash_msg);
        std::cerr << "Broadcasting robot crash msg";

        exit(signal_num);
    }
}

Thunderloop::Thunderloop(const robot_constants::RobotConstants& robot_constants,
                         const bool enable_log_merging, const int loop_hz)
    : toml_config_client_(std::make_unique<TomlConfigClient>(TOML_CONFIG_FILE_PATH)),
      robot_constants_(robot_constants),
      robot_id_(std::stoi(toml_config_client_->get(ROBOT_ID_CONFIG_KEY))),
      channel_id_(
          std::stoi(toml_config_client_->get(ROBOT_MULTICAST_CHANNEL_CONFIG_KEY))),
      network_interface_(toml_config_client_->get(ROBOT_NETWORK_INTERFACE_CONFIG_KEY)),
      loop_hz_(loop_hz),
      primitive_executor_(robot_constants)
{
    g3::overrideSetupSignals({});

    std::signal(SIGSEGV, tbotsExit);
    std::signal(SIGTERM, tbotsExit);
    std::signal(SIGABRT, tbotsExit);
    std::signal(SIGFPE, tbotsExit);
    std::signal(SIGINT, tbotsExit);
    std::signal(SIGILL, tbotsExit);

    // Initialize values for udp sender in signal handler
    robot_status      = &robot_status_;
    channel_id        = channel_id_;
    network_interface = network_interface_;
    robot_id          = robot_id_;

    NetworkLoggerSingleton::initializeLogger(robot_id_, enable_log_merging,
                                             network_interface_);

    const NetworkService::NetworkConfig network_config{
        .robot_id                 = static_cast<RobotId>(robot_id),
        .multicast_ip             = std::string(ROBOT_MULTICAST_CHANNELS.at(channel_id_)),
        .primitive_listener_port  = PRIMITIVE_PORT,
        .robot_status_sender_port = ROBOT_STATUS_PORT,
        .full_system_to_robot_ip_notification_port =
            FULL_SYSTEM_TO_ROBOT_IP_NOTIFICATION_PORT,
        .robot_to_full_system_ip_notification_port =
            ROBOT_TO_FULL_SYSTEM_IP_NOTIFICATION_PORT,
        .interface = network_interface,
    };

    network_service_ = std::make_unique<NetworkService>(network_config);
    LOG(INFO) << "THUNDERLOOP: Network Service initialized!";

#ifndef DISABLE_POWER_SERVICE
    power_service_ = std::make_unique<PowerService>(
        std::stod(toml_config_client_->get(ROBOT_KICK_EXP_COEFF_CONFIG_KEY)),
        std::stoi(toml_config_client_->get(ROBOT_KICK_CONSTANT_CONFIG_KEY)),
        std::stoi(toml_config_client_->get(ROBOT_CHIP_PULSE_WIDTH_CONFIG_KEY)));
    LOG(INFO) << "THUNDERLOOP: Power Service initialized!";
#else
    LOG(INFO) << "THUNDERLOOP: Power Service DISABLED!";
#endif

#ifndef DISABLE_MOTOR_SERVICE
    motor_service_  = std::make_unique<MotorService>(robot_constants);
    g_motor_service = motor_service_.get();
    motor_service_->setup();

    LOG(INFO) << "THUNDERLOOP: Motor Service initialized!";
#else
    LOG(INFO) << "THUNDERLOOP: Motor Service DISABLED!";
#endif

    imu_service_ = std::make_unique<ImuService>();
    LOG(INFO) << "THUNDERLOOP: IMU Service initialized!";

    LOG(INFO) << "THUNDERLOOP: finished initialization with ROBOT ID: " << robot_id_
              << ", CHANNEL ID: " << channel_id_
              << ", and NETWORK INTERFACE: " << network_interface_;
    LOG(INFO)
        << "THUNDERLOOP: to update Thunderloop configuration, edit TOML config file and restart Thunderloop";
}

/*
 * Run the main robot loop!
 *
 * Each iteration reads from the sensors and network, fuses them into a robot state
 * estimate, steps the active primitive, and drives the actuators. The body is kept
 * as a short sequence of named stages so the high-level control flow stays readable;
 * each stage is implemented in its own helper below.
 */
void Thunderloop::runLoop()
{
    const auto interval = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(loop_hz_)));

    auto prev_iter_start_time = std::chrono::steady_clock::now();
    auto next_shot            = prev_iter_start_time;

    // Initial version setup
    std::string thunderloop_hash, thunderloop_date_flashed;
    std::ifstream hashFile("~/thunderbots_hashes/thunderloop.hash");
    std::ifstream dateFile("~/thunderbots_hashes/thunderloop.date");
    std::getline(hashFile, thunderloop_hash);
    std::getline(dateFile, thunderloop_date_flashed);
    hashFile.close();
    dateFile.close();

    robot_status_.set_robot_id(robot_id_);
    robot_status_.set_thunderloop_version(thunderloop_hash);
    robot_status_.set_thunderloop_date_flashed(thunderloop_date_flashed);

    for (;;)
    {
        std::this_thread::sleep_until(next_shot);

        const auto iter_start_time      = std::chrono::steady_clock::now();
        const auto time_since_prev_iter = iter_start_time - prev_iter_start_time;
        const double time_since_prev_iter_s =
            std::chrono::duration<double>(time_since_prev_iter).count();

        FrameMarkStart(TracyConstants::THUNDERLOOP_FRAME_MARKER);

        robot_status_.clear_error_code();

        const std::optional<TbotsProto::Primitive> primitive =
            network_service_->poll(robot_status_);

        if (primitive.has_value())
        {
            primitive_executor_.updatePrimitive(primitive.value(), robot_status_);
        }

        const TbotsProto::DirectControlPrimitive direct_control_primitive =
            primitive_executor_.stepPrimitive(robot_status_, time_since_prev_iter_s);

        imu_service_->poll(robot_status_);

#ifndef DISABLE_MOTOR_SERVICE
        motor_service_->poll(direct_control_primitive, robot_status_,
                             time_since_prev_iter_s);
#endif

#ifndef DISABLE_POWER_SERVICE
        power_service_->poll(direct_control_primitive, robot_status_);
#endif

        FrameMarkEnd(TracyConstants::THUNDERLOOP_FRAME_MARKER);

        const auto iter_end_time = std::chrono::steady_clock::now();
        const auto iter_duration = iter_end_time - iter_start_time;
        robot_status_.mutable_thunderloop_status()->set_iteration_time_ms(
            std::chrono::duration<double, std::milli>(iter_duration).count());

        prev_iter_start_time = iter_start_time;
        next_shot += interval;

        if (next_shot < iter_end_time)
        {
            LOG(WARNING) << "Thunderloop iteration overran its "
                         << std::chrono::duration<double, std::milli>(interval).count()
                         << " ms interval, resetting loop schedule";
            next_shot = iter_end_time;
        }
    }
}
