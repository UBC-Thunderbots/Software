#include "software/embedded/thunderloop.h"

#include <Tracy.hpp>
#include <chrono>
#include <fstream>
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
#include "software/networking/tbots_network_exception.h"
#include "software/physics/velocity_conversion_util.h"
#include "software/tracy/tracy_constants.h"
#include "software/world/robot_state.h"

namespace
{
/**
 * Stuff that the signal handler needs to stop the motors and send a crash message.
 *
 * The signal handler must be a free function with C linkage, so it cannot reach
 * Thunderloop's members directly. Instead, we populate a global CrashContext during
 * Thunderloop initialization with everything necessary to handle a crash.
 */
struct CrashContext
{
    MotorService* motor_service           = nullptr;
    TbotsProto::RobotStatus* robot_status = nullptr;
    int channel_id                        = 0;
    std::string network_interface;
    RobotId robot_id = 0;
};

CrashContext crash_context;
}  // namespace

/**
 * Handles a termination signal. Stops the motors, sends a crash message with the
 * stack dump, and exits.
 *
 * Must have C linkage to be registered with std::signal.
 *
 * @param signal_num the signal number that triggered the handler
 */
extern "C" void tbotsExit(const int signal_num)
{
    if (crash_context.motor_service)
    {
        crash_context.motor_service->reset();
    }

    // g3log may have died due to the termination signal, so it isn't reliable to LOG
    std::cerr << "\n\n!!!\nReceived termination signal: " << g3::signalToStr(signal_num)
              << std::endl;
    std::cerr << "Thunderloop shutting down\n!!!\n" << std::endl;

    TbotsProto::RobotCrash crash_msg;
    const auto dump = g3::internal::stackdump();
    crash_msg.set_robot_id(crash_context.robot_id);
    crash_msg.set_stack_dump(dump);
    crash_msg.set_exit_signal(g3::signalToStr(signal_num));
    *(crash_msg.mutable_status()) = *crash_context.robot_status;

    auto sender = ThreadedProtoUdpSender<TbotsProto::RobotCrash>(
        std::string(ROBOT_MULTICAST_CHANNELS.at(crash_context.channel_id)),
        ROBOT_CRASH_PORT, crash_context.network_interface, true);
    sender.sendProto(crash_msg);
    std::cerr << "Broadcasting robot crash msg";

    exit(signal_num);
}

Thunderloop::Thunderloop(const robot_constants::RobotConstants& robot_constants,
                         const bool enable_log_merging, const int loop_hz)
    : toml_config_client_(std::make_unique<TomlConfigClient>(TOML_CONFIG_FILE_PATH)),
      loop_hz_(loop_hz)
{
    const RobotId robot_id = std::stoi(toml_config_client_->get(ROBOT_ID_CONFIG_KEY));
    const int channel_id =
        std::stoi(toml_config_client_->get(ROBOT_MULTICAST_CHANNEL_CONFIG_KEY));
    const std::string network_interface =
        toml_config_client_->get(ROBOT_NETWORK_INTERFACE_CONFIG_KEY);

    g3::overrideSetupSignals({});

    std::signal(SIGSEGV, tbotsExit);
    std::signal(SIGTERM, tbotsExit);
    std::signal(SIGABRT, tbotsExit);
    std::signal(SIGFPE, tbotsExit);
    std::signal(SIGINT, tbotsExit);
    std::signal(SIGILL, tbotsExit);

    // Initialize the crash context used by the signal handler
    crash_context.robot_status      = &robot_status_;
    crash_context.channel_id        = channel_id;
    crash_context.network_interface = network_interface;
    crash_context.robot_id          = robot_id;

    NetworkLoggerSingleton::initializeLogger(robot_id, enable_log_merging,
                                             network_interface);

    waitForNetworkUp(channel_id, network_interface);

    network_service_ = std::make_unique<NetworkService>(
        robot_id, std::string(ROBOT_MULTICAST_CHANNELS.at(channel_id)), PRIMITIVE_PORT,
        ROBOT_STATUS_PORT, FULL_SYSTEM_TO_ROBOT_IP_NOTIFICATION_PORT,
        ROBOT_TO_FULL_SYSTEM_IP_NOTIFICATION_PORT, ROBOT_LOGS_PORT, network_interface);
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
    motor_service_              = std::make_unique<MotorService>(robot_constants);
    crash_context.motor_service = motor_service_.get();
    motor_service_->setup();

    LOG(INFO) << "THUNDERLOOP: Motor Service initialized!";
#else
    LOG(INFO) << "THUNDERLOOP: Motor Service DISABLED!";
#endif

    imu_service_ = std::make_unique<ImuService>();

    robot_localizer_ =
        std::make_unique<RobotLocalizer>(RobotLocalizer::RobotLocalizerConfig{
            robot_constants.kalman_process_noise_variance_rad_per_s_4,
            robot_constants.kalman_vision_noise_variance_rad_2,
            robot_constants.kalman_motor_sensor_noise_variance_rad_per_s_2});

    primitive_executor_ = std::make_unique<PrimitiveExecutor>(robot_constants);

    // Initial version setup
    std::string thunderloop_hash, thunderloop_date_flashed;
    std::ifstream hashFile("~/thunderbots_hashes/thunderloop.hash");
    std::ifstream dateFile("~/thunderbots_hashes/thunderloop.date");
    std::getline(hashFile, thunderloop_hash);
    std::getline(dateFile, thunderloop_date_flashed);
    hashFile.close();
    dateFile.close();

    robot_status_.set_robot_id(robot_id);
    robot_status_.set_thunderloop_version(thunderloop_hash);
    robot_status_.set_thunderloop_date_flashed(thunderloop_date_flashed);

    LOG(INFO) << "THUNDERLOOP: finished initialization with ROBOT ID: " << robot_id
              << ", CHANNEL ID: " << channel_id
              << ", and NETWORK INTERFACE: " << network_interface;
    LOG(INFO)
        << "THUNDERLOOP: to update Thunderloop configuration, edit TOML config file and restart Thunderloop";
}

Thunderloop::~Thunderloop() {}

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

    last_primitive_received_time_ = std::chrono::steady_clock::now();

    for (;;)
    {
        std::this_thread::sleep_until(next_shot);

        const auto iter_start_time      = std::chrono::steady_clock::now();
        const auto time_since_prev_iter = iter_start_time - prev_iter_start_time;
        const Duration delta_time       = Duration::fromSeconds(
                  std::chrono::duration<double>(time_since_prev_iter).count());

        FrameMarkStart(TracyConstants::THUNDERLOOP_FRAME_MARKER);

        // Network Service: receive newest primitives and send out the last
        // robot status
        const NetworkPollResult network_result = pollNetwork();

        // Robot Localizer: fuse sensor measurements into a robot state estimate
        // and hand it to the primitive executor
        primitive_executor_->updateState(updateLocalization());

        // Primitive Executor: run the last primitive if we have not timed out,
        // producing the control command for this iteration
        const PrimitiveStepResult primitive_result = stepActivePrimitive(delta_time);

#ifndef DISABLE_MOTOR_SERVICE
        // Motor Service: execute the motor control command
        motor_service_->poll(primitive_result.direct_control, robot_status_, delta_time);
#endif

#ifndef DISABLE_POWER_SERVICE
        // Power Service: execute the power control command
        power_service_->poll(primitive_result.direct_control, robot_status_);
#endif

        // Robot Status: compose the per-stage results into the outgoing status
        assembleRobotStatus(network_result, primitive_result);

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

inline Thunderloop::NetworkPollResult Thunderloop::pollNetwork()
{
    NetworkPollResult result;
    TbotsProto::Primitive new_primitive;

    // Network Service: receive newest primitives and send out the last robot status
    const auto poll_start = std::chrono::steady_clock::now();

    ZoneNamedN(_tracy_network_poll, "Thunderloop: Poll NetworkService", true);

    new_primitive = network_service_->poll(robot_status_);

    const auto poll_end = std::chrono::steady_clock::now();
    result.poll_time_ms =
        std::chrono::duration<double, std::milli>(poll_end - poll_start).count();

    // Update the time elapsed since the last received primitive
    const auto time_since_last_primitive_received =
        std::chrono::steady_clock::now() - last_primitive_received_time_;
    result.network_status.set_ms_since_last_primitive_received(
        std::chrono::duration<double, std::milli>(time_since_last_primitive_received)
            .count());

    // If the primitive msg is new, update the internal buffer and start the new
    // primitive.
    if (new_primitive.time_sent().epoch_timestamp_seconds() >
        primitive_.time_sent().epoch_timestamp_seconds())
    {
        // Save new primitive
        primitive_ = new_primitive;

        // Feed the trajectory's starting pose to the localizer as a vision update.
        if (primitive_.has_move())
        {
            const Point position =
                createPoint(primitive_.move().xy_traj_params().start_position());
            const Angle orientation =
                createAngle(primitive_.move().w_traj_params().start_angle());

            robot_localizer_->update(
                RobotLocalizer::VisionData{position, orientation, RTT_S / 2});
        }

        last_primitive_received_time_ = std::chrono::steady_clock::now();

        // Start new primitive
        const auto start = std::chrono::steady_clock::now();
        primitive_executor_->updatePrimitive(primitive_);
        result.primitive_start_time_ms = std::chrono::duration<double, std::milli>(
                                             std::chrono::steady_clock::now() - start)
                                             .count();
    }

    return result;
}

inline RobotState Thunderloop::updateLocalization()
{
    const std::optional<ImuData> imu_poll = imu_service_->poll();

    // IMU: feed the measured angular velocity to the localizer
    if (imu_poll.has_value() && imu_poll->angular_velocity.has_value())
    {
        robot_localizer_->update(
            RobotLocalizer::ImuData{imu_poll->angular_velocity.value()});
    }

    // Motors: feed the measured wheel velocities (rotated into the global frame) to
    // the localizer
    if (robot_status_.has_motor_status())
    {
        const auto status = robot_status_.motor_status();

        robot_localizer_->update(RobotLocalizer::MotorData{
            localToGlobalVelocity(createVector(status.local_velocity()),
                                  robot_localizer_->getOrientation()),
            createAngularVelocity(status.angular_velocity())});
    }

    // Step the localizer forward using the measured linear acceleration
    Vector linear_acceleration;

#ifdef ENABLE_IMU_ACCEL
    if (imu_poll.has_value() && imu_poll->linear_acceleration.has_value())
    {
        const auto accel    = imu_poll->linear_acceleration.value();
        linear_acceleration = Vector(accel[0], accel[1]);
    }
#endif

    robot_localizer_->step(linear_acceleration);

    // Hand the fused state estimate to the primitive executor
    return RobotState(robot_localizer_->getPosition(), robot_localizer_->getVelocity(),
                      robot_localizer_->getOrientation(),
                      robot_localizer_->getAngularVelocity());
}

inline Thunderloop::PrimitiveStepResult Thunderloop::stepActivePrimitive(
    const Duration& delta_time)
{
    PrimitiveStepResult result;

    const auto poll_start = std::chrono::steady_clock::now();

    ZoneNamedN(_tracy_step_primitive, "Thunderloop: Step Primitive", true);

    // If primitive not received in a while, stop the robot
    const auto time_since_last_primitive_received =
        std::chrono::steady_clock::now() - last_primitive_received_time_;
    if (time_since_last_primitive_received >
        std::chrono::nanoseconds(static_cast<long>(PACKET_TIMEOUT_NS)))
    {
        primitive_executor_->updatePrimitive(*createStopPrimitiveProto());
    }

    result.direct_control =
        *primitive_executor_->stepPrimitive(result.executor_status, delta_time);

    const auto poll_end = std::chrono::steady_clock::now();
    result.step_time_ms =
        std::chrono::duration<double, std::milli>(poll_end - poll_start).count();

    return result;
}

inline void Thunderloop::assembleRobotStatus(const NetworkPollResult& network,
                                             const PrimitiveStepResult& primitive)
{
    // Fold the per-stage timing into the sticky telemetry. Fields whose stage did not run
    // this iteration (a new primitive start, a disabled service) keep their last value.
    thunderloop_status_.set_network_service_poll_time_ms(network.poll_time_ms);
    if (network.primitive_start_time_ms.has_value())
    {
        thunderloop_status_.set_primitive_executor_start_time_ms(
            network.primitive_start_time_ms.value());
    }
    thunderloop_status_.set_primitive_executor_step_time_ms(primitive.step_time_ms);

    TbotsProto::Timestamp time_sent;
    time_sent.set_epoch_timestamp_seconds(
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
            .count());

    // Compose the outgoing status. Note: motor_status and power_status are written into
    // robot_status_ directly by the motor/power services during their poll.
    robot_status_.set_last_handled_primitive_set(primitive_.sequence_number());
    *(robot_status_.mutable_time_sent())                 = time_sent;
    *(robot_status_.mutable_thunderloop_status())        = thunderloop_status_;
    *(robot_status_.mutable_network_status())            = network.network_status;
    *(robot_status_.mutable_primitive_executor_status()) = primitive.executor_status;
}

void Thunderloop::waitForNetworkUp(const int channel_id,
                                   const std::string& network_interface)
{
    std::unique_ptr<ThreadedUdpSender> network_tester;
    try
    {
        network_tester = std::make_unique<ThreadedUdpSender>(
            std::string(ROBOT_MULTICAST_CHANNELS.at(channel_id)), NETWORK_COMM_TEST_PORT,
            network_interface, true);
    }
    catch (TbotsNetworkException& e)
    {
        LOG(FATAL) << "Thunderloop cannot connect to the network. Error: " << e.what();
    }

    // Send an empty packet on the specific network interface to
    // ensure wifi is connected. Keeps trying until successful
    while (true)
    {
        try
        {
            network_tester->sendString("");
            break;
        }
        catch (std::exception& e)
        {
            // Resend the message after a delay
            LOG(WARNING) << "Thunderloop cannot connect to network!"
                         << "Waiting for connection...";
            sleep(PING_RETRY_DELAY_S);
        }
    }

    LOG(INFO) << "Thunderloop connected to network!";
}
