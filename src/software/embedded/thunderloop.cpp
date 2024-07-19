#include "software/embedded/thunderloop.h"

#include <Tracy.hpp>
#include <fstream>

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/robot_crash_msg.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/embedded/primitive_executor.h"
#include "software/embedded/services/motor.h"
#include "software/logger/logger.h"
#include "software/logger/network_logger.h"
#include "software/tracy/tracy_constants.h"
#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"
#include "software/world/robot_state.h"
#include "software/world/team.h"

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
        g_motor_service->resetMotorBoard();

        // by now g3log may have died due to the termination signal, so it isn't reliable
        // to log messages
        std::cerr << "\n\n!!!\nReceived termination signal: "
                  << g3::signalToStr(signal_num) << std::endl;
        std::cerr << "Thunderloop shutting down and motor board reset\n!!!\n"
                  << std::endl;

        TbotsProto::RobotCrash crash_msg;
        auto dump = g3::internal::stackdump();
        crash_msg.set_robot_id(robot_id);
        crash_msg.set_stack_dump(dump);
        crash_msg.set_exit_signal(g3::signalToStr(signal_num));
        *(crash_msg.mutable_status()) = *robot_status;

        std::optional<std::string> error;
        auto sender = std::make_unique<ThreadedProtoUdpSender<TbotsProto::RobotCrash>>(
            std::string(ROBOT_MULTICAST_CHANNELS.at(channel_id)), ROBOT_CRASH_PORT,
            network_interface, true, error);
        sender->sendProto(crash_msg);
        std::cerr << "Broadcasting robot crash msg";

        exit(signal_num);
    }
}

Thunderloop::Thunderloop(const RobotConstants_t& robot_constants, bool enable_log_merging,
                         const int loop_hz)
    // TODO (#2495): Set the friendly team colour
    : redis_client_(
          std::make_unique<RedisClient>(REDIS_DEFAULT_HOST, REDIS_DEFAULT_PORT)),
      motor_status_(std::nullopt),
      robot_constants_(robot_constants),
      robot_id_(std::stoi(redis_client_->getSync(ROBOT_ID_REDIS_KEY))),
      channel_id_(std::stoi(redis_client_->getSync(ROBOT_MULTICAST_CHANNEL_REDIS_KEY))),
      network_interface_(redis_client_->getSync(ROBOT_NETWORK_INTERFACE_REDIS_KEY)),
      loop_hz_(loop_hz),
      kick_coeff_(std::stod(redis_client_->getSync(ROBOT_KICK_EXP_COEFF_REDIS_KEY))),
      kick_constant_(std::stoi(redis_client_->getSync(ROBOT_KICK_CONSTANT_REDIS_KEY))),
      chip_pulse_width_(
          std::stoi(redis_client_->getSync(ROBOT_CHIP_PULSE_WIDTH_REDIS_KEY))),
      primitive_executor_(robot_constants, TeamColour::YELLOW, robot_id_)
{
    std::optional<std::string> network_test_error;
    ThreadedUdpSender network_test(ROBOT_MULTICAST_CHANNELS.at(channel_id_), UNUSED_PORT,
                                   network_interface_, true, network_test_error);

    // send an empty packet on the specific network interface to ensure wifi is connected,
    // keeps trying until success
    while (true)
    {
        try
        {
            network_test.sendString("");
            break;
        }
        catch (std::exception& e)
        {
            // resend the message after a delay
            std::cout
                << "Warning! Thunderloop cannot connect to network! Waiting for connection..."
                << std::endl;
            sleep(PING_RETRY_DELAY_S);
        }
    }

    std::cout << "Thunderloop connected to network!" << std::endl;

    g3::overrideSetupSignals({});
    NetworkLoggerSingleton::initializeLogger(channel_id_, network_interface_, robot_id_,
                                             enable_log_merging);

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
        std::string(ROBOT_MULTICAST_CHANNELS.at(channel_id_)), PRIMITIVE_PORT,
        ROBOT_STATUS_PORT, network_interface, true);
    LOG(INFO)
        << "THUNDERLOOP: Network Service initialized! Next initializing Power Service";

    power_service_ = std::make_unique<PowerService>();
    LOG(INFO)
        << "THUNDERLOOP: Power Service initialized! Next initializing Motor Service";

    motor_service_  = std::make_unique<MotorService>(robot_constants, loop_hz);
    g_motor_service = motor_service_.get();
    motor_service_->setup();
    LOG(INFO) << "THUNDERLOOP: Motor Service initialized!";

    LOG(INFO) << "THUNDERLOOP: finished initialization with ROBOT ID: " << robot_id_
              << ", CHANNEL ID: " << channel_id_
              << ", and NETWORK INTERFACE: " << network_interface_;
    LOG(INFO)
        << "THUNDERLOOP: to update Thunderloop configuration, change REDIS store and restart Thunderloop";
}

Thunderloop::~Thunderloop() {}

/*
 * Run the main robot loop!
 */
[[noreturn]] void Thunderloop::runLoop()
{
    // Timing
    struct timespec time_remaining_in_iteration;
    struct timespec poll_time;
    struct timespec iteration_time;
    struct timespec last_primitive_received_time;
    struct timespec last_world_received_time;
    struct timespec current_time;
    struct timespec last_chipper_fired;
    struct timespec last_kicker_fired;
    struct timespec prev_iter_start_time;

    // Input buffer
    TbotsProto::PrimitiveSet new_primitive_set;
    TbotsProto::World new_world;
    const TbotsProto::PrimitiveSet empty_primitive_set;

    // Loop interval
    int interval =
        static_cast<int>(1.0f / static_cast<float>(loop_hz_) * NANOSECONDS_PER_SECOND);

    time_remaining_in_iteration.tv_nsec = 0;
    time_remaining_in_iteration.tv_sec = 0;

    // Get current time
    // Note: CLOCK_MONOTONIC is used over CLOCK_REALTIME since
    // CLOCK_REALTIME can jump backwards
    clock_gettime(CLOCK_MONOTONIC, &last_primitive_received_time);
    clock_gettime(CLOCK_MONOTONIC, &last_world_received_time);
    clock_gettime(CLOCK_MONOTONIC, &last_chipper_fired);
    clock_gettime(CLOCK_MONOTONIC, &last_kicker_fired);
    clock_gettime(CLOCK_MONOTONIC, &prev_iter_start_time);

    for (;;)
    {
        struct timespec time_since_prev_iter;
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        ScopedTimespecTimer::timespecDiff(&current_time, &prev_iter_start_time,
                                          &time_since_prev_iter);
        double time_since_prev_iter_sec = getMilliseconds(time_since_prev_iter) * SECONDS_PER_MILLISECOND;
        prev_iter_start_time = current_time;
        {
            // Wait until next shot
            //
            // Note: CLOCK_MONOTONIC is used over CLOCK_REALTIME since
            // CLOCK_REALTIME can jump backwards
            // FLag of 0: relative sleep
            clock_nanosleep(CLOCK_MONOTONIC, 0, &time_remaining_in_iteration, NULL);

            FrameMarkStart(TracyConstants::THUNDERLOOP_FRAME_MARKER);

            ScopedTimespecTimer iteration_timer(&iteration_time);

            // Collect jetson status
            jetson_status_.set_cpu_temperature(getCpuTemperature());

            // Network Service: receive newest world, primitives and set out the last
            // robot status
            {
                ScopedTimespecTimer timer(&poll_time);

                ZoneNamedN(_tracy_network_poll, "Thunderloop: Poll NetworkService", true);

                new_primitive_set = network_service_->poll(robot_status_);
            }

            thunderloop_status_.set_network_service_poll_time_ms(
                getMilliseconds(poll_time));

            uint64_t last_handled_primitive_set = primitive_set_.sequence_number();

            // Updating primitives and world with newly received data
            // and setting the correct time elasped since last primitive / world

            struct timespec time_since_last_primitive_received;
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            ScopedTimespecTimer::timespecDiff(&current_time,
                                              &last_primitive_received_time,
                                              &time_since_last_primitive_received);
            network_status_.set_ms_since_last_primitive_received(
                getMilliseconds(time_since_last_primitive_received));

            // If the primitive msg is new, update the internal buffer
            // and start the new primitive.
            if (new_primitive_set.time_sent().epoch_timestamp_seconds() >
                primitive_set_.time_sent().epoch_timestamp_seconds())
            {
                // Save new primitive set
                primitive_set_ = new_primitive_set;

                // Update primitive executor's primitive set
                {
                    clock_gettime(CLOCK_MONOTONIC, &last_primitive_received_time);

                    // Start new primitive
                    {
                        ScopedTimespecTimer timer(&poll_time);
                        primitive_executor_.updatePrimitiveSet(primitive_set_);
                    }

                    thunderloop_status_.set_primitive_executor_start_time_ms(
                        getMilliseconds(poll_time));
                }
            }

            if (motor_status_.has_value())
            {
                auto status = motor_status_.value();
                primitive_executor_.updateVelocity(
                    createVector(status.local_velocity()),
                    createAngularVelocity(status.angular_velocity()));
            }

            // Timeout Overrides for Primitives
            // These should be after the new primitive update section above

            // If primitive not received in a while, stop robot
            // Primitive Executor: run the last primitive if we have not timed out
            {
                ScopedTimespecTimer timer(&poll_time);

                ZoneNamedN(_tracy_step_primitive, "Thunderloop: Step Primitive", true);

                // Handle emergency stop override
                auto nanoseconds_elapsed_since_last_primitive =
                    getNanoseconds(time_since_last_primitive_received);

                if (nanoseconds_elapsed_since_last_primitive > PACKET_TIMEOUT_NS)
                {
                    primitive_executor_.setStopPrimitive();
                }

                direct_control_ =
                    *primitive_executor_.stepPrimitive(time_since_prev_iter_sec, primitive_executor_status_);
            }

            thunderloop_status_.set_primitive_executor_step_time_ms(
                getMilliseconds(poll_time));

            // Power Service: execute the power control command
            {
                ScopedTimespecTimer timer(&poll_time);

                ZoneNamedN(_tracy_power_service_poll, "Thunderloop: Poll PowerService",
                           true);

                power_status_ =
                    power_service_->poll(direct_control_.power_control(), kick_coeff_,
                                         kick_constant_, chip_pulse_width_);
            }
            thunderloop_status_.set_power_service_poll_time_ms(
                getMilliseconds(poll_time));

            struct timespec time_since_kicker_fired;
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            ScopedTimespecTimer::timespecDiff(&current_time, &last_kicker_fired,
                                              &time_since_kicker_fired);
            chipper_kicker_status_.set_ms_since_kicker_fired(
                getMilliseconds(time_since_kicker_fired));

            struct timespec time_since_chipper_fired;
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            ScopedTimespecTimer::timespecDiff(&current_time, &last_chipper_fired,
                                              &time_since_chipper_fired);
            chipper_kicker_status_.set_ms_since_chipper_fired(
                getMilliseconds(time_since_chipper_fired));

            // if a kick proto is sent or if autokick is on
            if (direct_control_.power_control().chicker().has_kick_speed_m_per_s() ||
                direct_control_.power_control()
                    .chicker()
                    .auto_chip_or_kick()
                    .has_autokick_speed_m_per_s())
            {
                clock_gettime(CLOCK_MONOTONIC, &last_kicker_fired);
            }
            // if a chip proto is sent or if autochip is on
            else if (direct_control_.power_control()
                         .chicker()
                         .has_chip_distance_meters() ||
                     direct_control_.power_control()
                         .chicker()
                         .auto_chip_or_kick()
                         .has_autochip_distance_meters())
            {
                clock_gettime(CLOCK_MONOTONIC, &last_chipper_fired);
            }

            // Motor Service: execute the motor control command
            {
                ScopedTimespecTimer timer(&poll_time);

                ZoneNamedN(_tracy_motor_service, "Thunderloop: Poll MotorService", true);
                motor_status_ = motor_service_->poll(direct_control_.motor_control(),
                                                     time_since_prev_iter_sec);
            }
            thunderloop_status_.set_motor_service_poll_time_ms(
                getMilliseconds(poll_time));

            clock_gettime(CLOCK_MONOTONIC, &current_time);
            time_sent_.set_epoch_timestamp_seconds(
                static_cast<double>(current_time.tv_sec));

            // Update Robot Status with poll responses
            robot_status_.set_robot_id(robot_id_);
            robot_status_.set_last_handled_primitive_set(last_handled_primitive_set);
            *(robot_status_.mutable_time_sent())             = time_sent_;
            *(robot_status_.mutable_thunderloop_status())    = thunderloop_status_;
            *(robot_status_.mutable_motor_status())          = motor_status_.value();
            *(robot_status_.mutable_power_status())          = power_status_;
            *(robot_status_.mutable_jetson_status())         = jetson_status_;
            *(robot_status_.mutable_network_status())        = network_status_;
            *(robot_status_.mutable_chipper_kicker_status()) = chipper_kicker_status_;
            *(robot_status_.mutable_primitive_executor_status()) =
                primitive_executor_status_;

            // Update Redis
            {
                ZoneNamedN(_tracy_redis, "Thunderloop: Commit to REDIS", true);

                redis_client_->setNoCommit(
                    ROBOT_BATTERY_VOLTAGE_REDIS_KEY,
                    std::to_string(power_status_.battery_voltage()));
                redis_client_->setNoCommit(ROBOT_CURRENT_DRAW_REDIS_KEY,
                                           std::to_string(power_status_.current_draw()));
                redis_client_->asyncCommit();
            }

            updateErrorCodes();
        }

        auto loop_duration_ns = getNanoseconds(iteration_time);
        thunderloop_status_.set_iteration_time_ms(loop_duration_ns /
                                                  NANOSECONDS_PER_MILLISECOND);

        // Calculate next shot taking into account how long this iteration took
        time_remaining_in_iteration.tv_nsec = interval - static_cast<long int>(loop_duration_ns);
        timespecNorm(time_remaining_in_iteration);

        FrameMarkEnd(TracyConstants::THUNDERLOOP_FRAME_MARKER);
    }
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

double Thunderloop::getCpuTemperature()
{
    // Get the CPU temperature
    std::ifstream cpu_temp_file(CPU_TEMP_FILE_PATH);
    if (cpu_temp_file.is_open())
    {
        std::string cpu_temp_str;
        std::getline(cpu_temp_file, cpu_temp_str);
        cpu_temp_file.close();

        // Convert the temperature to a double
        // The temperature returned is in millicelcius
        double cpu_temp = std::stod(cpu_temp_str) / 1000.0;
        return cpu_temp;
    }
    else
    {
        LOG(WARNING) << "Could not open CPU temperature file";
        return 0.0;
    }
}

bool isPowerStable(std::ifstream& log_file)
{
    // @TODO #3248 : add isPowerStable support to the raspberry pi
    // For now in robocup, we return true since we cannot really access the dmesg. This
    // would be addressed in a future PR
    if (PLATFORM == Platform::RASP_PI)
    {
        return true;
    }
    // the following code is executed for the jetson and not the raspberry pi

    // if the log file cannot be open, we would return false. Chances are, the battery
    // power supply is indeed stable
    if (!log_file.is_open())
    {
        LOG(WARNING) << "Cannot dmesg log file. Do you have permission?";
        return true;
    }

    std::string line;
    while (std::getline(log_file, line))
    {
        // if this lines exist, we know for sure that the battery is not stable!
        if (line.find("soctherm: OC ALARM 0x00000001") != std::string::npos)
        {
            return false;
        }
    }

    // We have reached the end of the line with the while loop from above. Therefore, we
    // need to run std::ifstream::clear so that std::getline would return the new lines in
    // the file stream.
    log_file.clear();

    return true;
}

void Thunderloop::updateErrorCodes()
{
    // Clear existing codes
    robot_status_.clear_error_code();

    // Updates error status
    if (power_status_.battery_voltage() <= BATTERY_WARNING_VOLTAGE)
    {
        robot_status_.mutable_error_code()->Add(TbotsProto::ErrorCode::LOW_BATTERY);
    }
    if (power_status_.capacitor_voltage() >= MAX_CAPACITOR_VOLTAGE)
    {
        robot_status_.mutable_error_code()->Add(TbotsProto::ErrorCode::HIGH_CAP);
    }
    if (jetson_status_.cpu_temperature() >= MAX_JETSON_TEMP_C)
    {
        robot_status_.mutable_error_code()->Add(TbotsProto::ErrorCode::HIGH_BOARD_TEMP);
    }

    if (!isPowerStable(log_file))
    {
        robot_status_.mutable_error_code()->Add(
            TbotsProto::ErrorCode::UNSTABLE_POWER_SUPPLY);
    }
}
