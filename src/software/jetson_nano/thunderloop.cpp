#include "software/jetson_nano/thunderloop.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/jetson_nano/services/motor.h"
#include "software/logger/logger.h"
#include "software/logger/network_logger.h"
#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"
#include "software/world/robot_state.h"
#include "software/world/team.h"

/**
 * https://rt.wiki.kernel.org/index.php/Squarewave-example
 * using clock_nanosleep of librt
 */
extern int clock_nanosleep(clockid_t __clock_id, int __flags,
                           __const struct timespec* __req, struct timespec* __rem);

Thunderloop::Thunderloop(const RobotConstants_t& robot_constants, const int loop_hz)
    // TODO (#2495): Set the friendly team colour once we receive World proto
    : redis_client_(
          std::make_unique<RedisClient>(REDIS_DEFAULT_HOST, REDIS_DEFAULT_PORT)),
      robot_constants_(robot_constants),
      robot_id_(std::stoi(redis_client_->getSync(ROBOT_ID_REDIS_KEY))),
      channel_id_(std::stoi(redis_client_->getSync(ROBOT_MULTICAST_CHANNEL_REDIS_KEY))),
      network_interface_(redis_client_->getSync(ROBOT_NETWORK_INTERFACE_REDIS_KEY)),
      loop_hz_(loop_hz),
      kick_slope_(std::stoi(redis_client_->getSync(ROBOT_KICK_SLOPE_REDIS_KEY))),
      kick_constant_(std::stoi(redis_client_->getSync(ROBOT_KICK_CONSTANT_REDIS_KEY))),
      chip_pulse_width_(
          std::stoi(redis_client_->getSync(ROBOT_CHIP_PULSE_WIDTH_REDIS_KEY))),
      primitive_executor_(loop_hz, robot_constants, TeamColour::YELLOW, robot_id_)
{
    NetworkLoggerSingleton::initializeLogger(channel_id_, network_interface_, robot_id_);
    LOG(INFO)
        << "THUNDERLOOP: Network Logger initialized! Next initializing Network Service";

    network_service_ = std::make_unique<NetworkService>(
        std::string(ROBOT_MULTICAST_CHANNELS.at(channel_id_)) + "%" + network_interface_,
        VISION_PORT, PRIMITIVE_PORT, ROBOT_STATUS_PORT, true);
    LOG(INFO)
        << "THUNDERLOOP: Network Service initialized! Next initializing Motor Service";

    motor_service_ = std::make_unique<MotorService>(robot_constants, loop_hz);
    LOG(INFO)
        << "THUNDERLOOP: Motor Service initialized! Next initializing Power Service";

    power_service_ = std::make_unique<PowerService>();
    LOG(INFO) << "THUNDERLOOP: Power Service initialized!";

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
    struct timespec next_shot;
    struct timespec poll_time;
    struct timespec iteration_time;
    struct timespec last_primitive_received_time;
    struct timespec last_world_recieved_time;
    struct timespec current_time;
    struct timespec last_chipper_fired;
    struct timespec last_kicker_fired;

    // Input buffer
    TbotsProto::PrimitiveSet new_primitive_set;
    TbotsProto::World new_world;
    const TbotsProto::PrimitiveSet empty_primitive_set;

    // Loop interval
    int interval =
        static_cast<int>(1.0f / static_cast<float>(loop_hz_) * NANOSECONDS_PER_SECOND);

    // Get current time
    // Note: CLOCK_MONOTONIC is used over CLOCK_REALTIME since
    // CLOCK_REALTIME can jump backwards
    clock_gettime(CLOCK_MONOTONIC, &next_shot);
    clock_gettime(CLOCK_MONOTONIC, &last_primitive_received_time);
    clock_gettime(CLOCK_MONOTONIC, &last_world_recieved_time);
    clock_gettime(CLOCK_MONOTONIC, &last_chipper_fired);
    clock_gettime(CLOCK_MONOTONIC, &last_kicker_fired);

    double loop_duration_seconds = 0.0;

    for (;;)
    {
        {
            // Wait until next shot
            //
            // Note: CLOCK_MONOTONIC is used over CLOCK_REALTIME since
            // CLOCK_REALTIME can jump backwards
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_shot, NULL);
            ScopedTimespecTimer iteration_timer(&iteration_time);

            // Collect jetson status
            jetson_status_.set_cpu_temperature(getCpuTemperature());

            // Network Service: receive newest world, primitives and set out the last
            // robot status
            {
                ScopedTimespecTimer timer(&poll_time);
                auto result       = network_service_->poll(robot_status_);
                new_primitive_set = std::get<0>(result);
                new_world         = std::get<1>(result);
            }

            thunderloop_status_.set_network_service_poll_time_ns(
                static_cast<unsigned long>(poll_time.tv_nsec));

            network_status_.set_ms_since_last_primitive_received(UINT64_MAX);
            network_status_.set_ms_since_last_vision_received(UINT64_MAX);

            uint64_t last_handled_primitive_set = primitive_set_.sequence_number();

            // If the primitive msg is new, update the internal buffer
            // and start the new primitive.
            if (new_primitive_set.time_sent().epoch_timestamp_seconds() >
                primitive_set_.time_sent().epoch_timestamp_seconds())
            {
                // Save new primitive set
                primitive_set_ = new_primitive_set;

                // Update primitive executor's primitive set
                {
                    struct timespec time_since_last_primitive_received;
                    clock_gettime(CLOCK_MONOTONIC, &current_time);
                    ScopedTimespecTimer::timespecDiff(
                            &current_time, &last_primitive_received_time, &time_since_last_primitive_received);
                    clock_gettime(CLOCK_MONOTONIC, &last_primitive_received_time);
                    network_status_.set_ms_since_last_primitive_received(
                            time_since_last_primitive_received.tv_nsec / 1000000);

                    // Start new primitive
                    {
                        ScopedTimespecTimer timer(&poll_time);
                        primitive_executor_.updatePrimitiveSet(primitive_set_);
                    }

                    thunderloop_status_.set_primitive_executor_start_time_ns(
                        static_cast<unsigned long>(poll_time.tv_nsec));
                }
            }

            // If the world msg is new, update the internal buffer
            if (new_world.time_sent().epoch_timestamp_seconds() >
                world_.time_sent().epoch_timestamp_seconds())
            {
                struct timespec time_since_last_vision_received;
                clock_gettime(CLOCK_MONOTONIC, &current_time);
                ScopedTimespecTimer::timespecDiff(&current_time, &last_world_recieved_time,
                                                  &time_since_last_vision_received);
                clock_gettime(CLOCK_MONOTONIC, &last_world_recieved_time);
                network_status_.set_ms_since_last_vision_received(time_since_last_vision_received.tv_nsec / 1000000);
                primitive_executor_.updateWorld(new_world);
                world_ = new_world;
            }

            // If world not received in a while, stop robot
            // Primitive Executor: run the last primitive if we have not timed out
            {
                ScopedTimespecTimer timer(&poll_time);

                // Handle emergency stop override
                auto nanoseconds_elapsed_since_last_primitive =
                        time_since_last_primitive_received.tv_sec * static_cast<int>(NANOSECONDS_PER_SECOND) +
                        time_since_last_primitive_received.tv_nsec;

                if (nanoseconds_elapsed_since_last_primitive >
                    static_cast<long>(PRIMITIVE_MANAGER_TIMEOUT_NS))
                {
                    primitive_executor_.setStopPrimitive();

                    // Log milliseconds since last world received if we are timing out
                    LOG(WARNING)
                        << "Primitive timeout, overriding with StopPrimitive\n"
                        << "Milliseconds since last world: "
                        << static_cast<int>(nanoseconds_elapsed_since_last_primitive) *
                               MILLISECONDS_PER_NANOSECOND;
                }

                direct_control_ = *primitive_executor_.stepPrimitive();
            }

            thunderloop_status_.set_primitive_executor_step_time_ns(
                static_cast<unsigned long>(poll_time.tv_nsec));

            // Power Service: execute the power control command
            {
                ScopedTimespecTimer timer(&poll_time);
                power_status_ =
                    power_service_->poll(direct_control_.power_control(), kick_slope_,
                                         kick_constant_, chip_pulse_width_);
            }
            thunderloop_status_.set_power_service_poll_time_ns(
                static_cast<unsigned long>(poll_time.tv_nsec));

            chipper_kicker_status_.set_ms_since_chipper_fired(UINT64_MAX);
            chipper_kicker_status_.set_ms_since_kicker_fired(UINT64_MAX);

            if (direct_control_.power_control().chicker().has_kick_speed_m_per_s())
            {
                struct timespec time_since_kicker_fired;
                clock_gettime(CLOCK_MONOTONIC, &current_time);
                ScopedTimespecTimer::timespecDiff(&current_time, &last_kicker_fired,
                                                  &time_since_kicker_fired);
                clock_gettime(CLOCK_MONOTONIC, &last_kicker_fired);
                chipper_kicker_status_.set_ms_since_kicker_fired(time_since_kicker_fired.tv_nsec / 1000000);
            }
            else if (direct_control_.power_control().chicker().has_chip_distance_meters())
            {
                struct timespec time_since_chipper_fired;
                clock_gettime(CLOCK_MONOTONIC, &current_time);
                ScopedTimespecTimer::timespecDiff(&current_time, &last_chipper_fired,
                                                  &time_since_chipper_fired);
                clock_gettime(CLOCK_MONOTONIC, &last_chipper_fired);
                chipper_kicker_status_.set_ms_since_kicker_fired(time_since_chipper_fired.tv_nsec / 1000000);
            }

            // Motor Service: execute the motor control command
            {
                ScopedTimespecTimer timer(&poll_time);
                motor_status_ = motor_service_->poll(direct_control_.motor_control(),
                                                     loop_duration_seconds);
                primitive_executor_.updateVelocity(
                    createVector(motor_status_.local_velocity()),
                    createAngularVelocity(motor_status_.angular_velocity()));
            }
            thunderloop_status_.set_motor_service_poll_time_ns(
                static_cast<unsigned long>(poll_time.tv_nsec));

            clock_gettime(CLOCK_MONOTONIC, &current_time);
            time_sent_.set_epoch_timestamp_seconds(current_time.tv_sec);

            // Update Robot Status with poll responses
            robot_status_.set_robot_id(robot_id_);
            robot_status_.set_last_handled_primitive_set(last_handled_primitive_set);
            *(robot_status_.mutable_time_sent())                = time_sent_;
            *(robot_status_.mutable_thunderloop_status())       = thunderloop_status_;
            *(robot_status_.mutable_motor_status())             = motor_status_;
            *(robot_status_.mutable_power_status())             = power_status_;
            *(robot_status_.mutable_jetson_status())            = jetson_status_;
            *(robot_status_.mutable_network_status())           = network_status_;
            *(robot_status_.mutable_chipper_kicker_status())    = chipper_kicker_status_;

            // Update Redis
            redis_client_->setNoCommit(ROBOT_BATTERY_VOLTAGE_REDIS_KEY,
                                       std::to_string(power_status_.battery_voltage()));
            redis_client_->setNoCommit(ROBOT_CURRENT_DRAW_REDIS_KEY,
                                       std::to_string(power_status_.current_draw()));
            redis_client_->asyncCommit();
        }

        auto loop_duration =
            iteration_time.tv_sec * static_cast<int>(NANOSECONDS_PER_SECOND) +
            iteration_time.tv_nsec;
        thunderloop_status_.set_iteration_time_ns(loop_duration);

        // Make sure the iteration can fit inside the period of the loop
        loop_duration_seconds =
            static_cast<double>(loop_duration) * SECONDS_PER_NANOSECOND;

        // Calculate next shot taking into account how long this iteration took
        next_shot.tv_nsec += interval - loop_duration;
        timespecNorm(next_shot);
    }
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
