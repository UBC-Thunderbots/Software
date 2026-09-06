#include "software/embedded/thunderloop.h"

#include <Tracy.hpp>
#include <fstream>

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
#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"
#include "software/world/robot_state.h"

/**
 * https://web.archive.org/web/20210308013218/https://rt.wiki.kernel.org/index.php/Squarewave-example
 * using clock_nanosleep of librt
 */
extern int clock_nanosleep(clockid_t __clock_id, int __flags,
                           __const struct timespec* __req, struct timespec* __rem);

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
                         bool enable_log_merging, const int loop_hz)
    : toml_config_client_(std::make_unique<TomlConfigClient>(TOML_CONFIG_FILE_PATH)),
      robot_constants_(robot_constants),
      robot_id_(std::stoi(toml_config_client_->get(ROBOT_ID_CONFIG_KEY))),
      channel_id_(
          std::stoi(toml_config_client_->get(ROBOT_MULTICAST_CHANNEL_CONFIG_KEY))),
      network_interface_(toml_config_client_->get(ROBOT_NETWORK_INTERFACE_CONFIG_KEY)),
      loop_hz_(loop_hz),
      primitive_executor_(robot_constants),
      robot_localizer_(RobotLocalizer::RobotLocalizerConfig{
          robot_constants.kalman_process_noise_variance_rad_per_s_4,
          robot_constants.kalman_vision_noise_variance_rad_2,
          robot_constants.kalman_motor_sensor_noise_variance_rad_per_s_2})
{
    waitForNetworkUp();

    g3::overrideSetupSignals({});
    NetworkLoggerSingleton::initializeLogger(robot_id_, enable_log_merging,
                                             network_interface_);

    // catch all catch-able signals
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

    LOG(INFO)
        << "THUNDERLOOP: Network Logger initialized! Next initializing Network Service";

    network_service_ = std::make_unique<NetworkService>(
        robot_id, std::string(ROBOT_MULTICAST_CHANNELS.at(channel_id_)), PRIMITIVE_PORT,
        ROBOT_STATUS_PORT, FULL_SYSTEM_TO_ROBOT_IP_NOTIFICATION_PORT,
        ROBOT_TO_FULL_SYSTEM_IP_NOTIFICATION_PORT, ROBOT_LOGS_PORT, network_interface);
    LOG(INFO)
        << "THUNDERLOOP: Network Service initialized! Next initializing Power Service";

#ifndef DISABLE_POWER_SERVICE
    power_service_ = std::make_unique<PowerService>(
        std::stod(toml_config_client_->get(ROBOT_KICK_EXP_COEFF_CONFIG_KEY)),
        std::stoi(toml_config_client_->get(ROBOT_KICK_CONSTANT_CONFIG_KEY)),
        std::stoi(toml_config_client_->get(ROBOT_CHIP_PULSE_WIDTH_CONFIG_KEY)));
    LOG(INFO)
        << "THUNDERLOOP: Power Service initialized! Next initializing Motor Service";
#else
    LOG(INFO) << "THUNDERLOOP: Power Service DISABLED! Next initializing Motor Service";
#endif

#ifndef DISABLE_MOTOR_SERVICE
    motor_service_  = std::make_unique<MotorService>(robot_constants);
    g_motor_service = motor_service_.get();
    motor_service_->setup();

    LOG(INFO) << "THUNDERLOOP: Motor Service initialized! Next initializing IMU Service";
#else
    LOG(INFO) << "THUNDERLOOP: Motor Service DISABLED! Next initializing IMU Service";
#endif

    imu_service_ = std::make_unique<ImuService>();

    LOG(INFO) << "THUNDERLOOP: finished initialization with ROBOT ID: " << robot_id_
              << ", CHANNEL ID: " << channel_id_
              << ", and NETWORK INTERFACE: " << network_interface_;
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
    // Timing local to the loop. Cross-iteration timing state lives in members
    // (last_primitive_received_time_, last_chipper_fired_, last_kicker_fired_) so the
    // stage helpers can read and update it without threading it through their signatures.
    struct timespec next_shot;
    struct timespec iteration_time;
    struct timespec current_time;
    struct timespec prev_iter_start_time;

    // Loop interval
    int interval =
        static_cast<int>(1.0f / static_cast<float>(loop_hz_) * NANOSECONDS_PER_SECOND);

    // Get current time
    // Note: CLOCK_MONOTONIC is used over CLOCK_REALTIME since
    // CLOCK_REALTIME can jump backwards
    clock_gettime(CLOCK_MONOTONIC, &next_shot);
    clock_gettime(CLOCK_MONOTONIC, &last_primitive_received_time_);
    clock_gettime(CLOCK_MONOTONIC, &last_chipper_fired_);
    clock_gettime(CLOCK_MONOTONIC, &last_kicker_fired_);
    clock_gettime(CLOCK_MONOTONIC, &prev_iter_start_time);

    // Initial version setup
    std::string thunderloop_hash, thunderloop_date_flashed;
    std::ifstream hashFile("~/thunderbots_hashes/thunderloop.hash");
    std::ifstream dateFile("~/thunderbots_hashes/thunderloop.date");
    std::getline(hashFile, thunderloop_hash);
    std::getline(dateFile, thunderloop_date_flashed);
    hashFile.close();
    dateFile.close();

    robot_status_.set_thunderloop_version(thunderloop_hash);
    robot_status_.set_thunderloop_date_flashed(thunderloop_date_flashed);
    *(robot_status_.mutable_motor_status()) = TbotsProto::MotorStatus();
    *(robot_status_.mutable_power_status()) = TbotsProto::PowerStatus();

    for (;;)
    {
        struct timespec time_since_prev_iter;
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        ScopedTimespecTimer::timespecDiff(&current_time, &prev_iter_start_time,
                                          &time_since_prev_iter);
        prev_iter_start_time = current_time;
        {
            // Wait until next shot
            //
            // Note: CLOCK_MONOTONIC is used over CLOCK_REALTIME since
            // CLOCK_REALTIME can jump backwards
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_shot, NULL);

            FrameMarkStart(TracyConstants::THUNDERLOOP_FRAME_MARKER);

            ScopedTimespecTimer iteration_timer(&iteration_time);

            const Duration delta_time = Duration::fromSeconds(
                getMilliseconds(time_since_prev_iter) * SECONDS_PER_MILLISECOND);

            // Network Service: receive newest primitives and send out the last
            // robot status
            const NetworkPollResult network_result = pollNetwork();

            // Robot Localizer: fuse sensor measurements into a robot state estimate
            // and hand it to the primitive executor
            primitive_executor_.updateState(updateLocalization());

            // Primitive Executor: run the last primitive if we have not timed out,
            // producing the control command for this iteration
            const PrimitiveStepResult primitive_result = stepActivePrimitive(delta_time);

#ifndef DISABLE_MOTOR_SERVICE
            // Motor Service: execute the motor control command
            motor_service_->poll(primitive_result.direct_control, robot_status_,
                                 delta_time);
#endif

#ifndef DISABLE_POWER_SERVICE
            // Power Service: execute the power control command
            power_service_->poll(primitive_result.direct_control, robot_status_);
#endif

            // Robot Status: compose the per-stage results into the outgoing status
            assembleRobotStatus(network_result, primitive_result);
        }

        auto loop_duration_ns = getNanoseconds(iteration_time);
        thunderloop_status_.set_iteration_time_ms(loop_duration_ns /
                                                  NANOSECONDS_PER_MILLISECOND);

        // Calculate next shot (which is an absolute time)
        next_shot.tv_nsec += interval;
        timespecNorm(next_shot);

        FrameMarkEnd(TracyConstants::THUNDERLOOP_FRAME_MARKER);
    }
}

inline Thunderloop::NetworkPollResult Thunderloop::pollNetwork()
{
    NetworkPollResult result;
    struct timespec poll_time;
    struct timespec current_time;
    TbotsProto::Primitive new_primitive;

    // Network Service: receive newest primitives and send out the last robot status
    {
        ScopedTimespecTimer timer(&poll_time);

        ZoneNamedN(_tracy_network_poll, "Thunderloop: Poll NetworkService", true);

        new_primitive = network_service_->poll(robot_status_);
    }

    result.poll_time_ms = getMilliseconds(poll_time);

    // Update the time elapsed since the last received primitive
    struct timespec time_since_last_primitive_received;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    ScopedTimespecTimer::timespecDiff(&current_time, &last_primitive_received_time_,
                                      &time_since_last_primitive_received);
    result.network_status.set_ms_since_last_primitive_received(
        getMilliseconds(time_since_last_primitive_received));

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

            robot_localizer_.update(
                RobotLocalizer::VisionData{position, orientation, RTT_S / 2});
        }

        clock_gettime(CLOCK_MONOTONIC, &last_primitive_received_time_);

        // Start new primitive
        struct timespec start_time;
        {
            ScopedTimespecTimer timer(&start_time);
            primitive_executor_.updatePrimitive(primitive_);
        }

        result.primitive_start_time_ms = getMilliseconds(start_time);
    }

    return result;
}

inline RobotState Thunderloop::updateLocalization()
{
    const std::optional<ImuData> imu_poll = imu_service_->poll();

    // IMU: feed the measured angular velocity to the localizer
    if (imu_poll.has_value() && imu_poll->angular_velocity.has_value())
    {
        robot_localizer_.update(
            RobotLocalizer::ImuData{imu_poll->angular_velocity.value()});
    }

    // Motors: feed the measured wheel velocities (rotated into the global frame) to
    // the localizer
    if (robot_status_.has_motor_status())
    {
        const auto status = robot_status_.motor_status();

        robot_localizer_.update(RobotLocalizer::MotorData{
            localToGlobalVelocity(createVector(status.local_velocity()),
                                  robot_localizer_.getOrientation()),
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

    robot_localizer_.step(linear_acceleration);

    // Hand the fused state estimate to the primitive executor
    return RobotState(robot_localizer_.getPosition(), robot_localizer_.getVelocity(),
                      robot_localizer_.getOrientation(),
                      robot_localizer_.getAngularVelocity());
}

inline Thunderloop::PrimitiveStepResult Thunderloop::stepActivePrimitive(
    const Duration& delta_time)
{
    PrimitiveStepResult result;
    struct timespec poll_time;
    struct timespec current_time;
    struct timespec time_since_last_primitive_received;

    clock_gettime(CLOCK_MONOTONIC, &current_time);
    ScopedTimespecTimer::timespecDiff(&current_time, &last_primitive_received_time_,
                                      &time_since_last_primitive_received);

    {
        ScopedTimespecTimer timer(&poll_time);

        ZoneNamedN(_tracy_step_primitive, "Thunderloop: Step Primitive", true);

        // If primitive not received in a while, stop the robot
        auto nanoseconds_elapsed_since_last_primitive =
            getNanoseconds(time_since_last_primitive_received);

        if (nanoseconds_elapsed_since_last_primitive > PACKET_TIMEOUT_NS)
        {
            primitive_executor_.updatePrimitive(*createStopPrimitiveProto());
        }

        result.direct_control =
            *primitive_executor_.stepPrimitive(result.executor_status, delta_time);
    }

    result.step_time_ms = getMilliseconds(poll_time);

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

    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    TbotsProto::Timestamp time_sent;
    time_sent.set_epoch_timestamp_seconds(static_cast<double>(current_time.tv_sec));

    // Compose the outgoing status. Note: motor_status and power_status are written into
    // robot_status_ directly by the motor/power services during their poll.
    robot_status_.set_robot_id(robot_id_);
    robot_status_.set_last_handled_primitive_set(primitive_.sequence_number());
    *(robot_status_.mutable_time_sent())                 = time_sent;
    *(robot_status_.mutable_thunderloop_status())        = thunderloop_status_;
    *(robot_status_.mutable_network_status())            = network.network_status;
    *(robot_status_.mutable_primitive_executor_status()) = primitive.executor_status;
}

double Thunderloop::getMilliseconds(timespec time)
{
    return (static_cast<double>(time.tv_sec) * MILLISECONDS_PER_SECOND) +
           (static_cast<double>(time.tv_nsec) / NANOSECONDS_PER_MILLISECOND);
}

double Thunderloop::getNanoseconds(timespec time)
{
    return (static_cast<double>(time.tv_sec) * NANOSECONDS_PER_SECOND) +
           static_cast<double>(time.tv_nsec);
}

void Thunderloop::timespecNorm(struct timespec& ts)
{
    while (ts.tv_nsec >= static_cast<int>(NANOSECONDS_PER_SECOND))
    {
        ts.tv_nsec -= static_cast<int>(NANOSECONDS_PER_SECOND);
        ts.tv_sec++;
    }
}

void Thunderloop::waitForNetworkUp()
{
    std::unique_ptr<ThreadedUdpSender> network_tester;
    try
    {
        network_tester = std::make_unique<ThreadedUdpSender>(
            std::string(ROBOT_MULTICAST_CHANNELS.at(channel_id_)), NETWORK_COMM_TEST_PORT,
            network_interface_, true);
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
