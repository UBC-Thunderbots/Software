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
    : robot_id_(MAX_ROBOT_IDS + 1),  // Initialize to a robot ID that is not valid
      primitive_executor_(1.0 / loop_hz, robot_constants, TeamColour::YELLOW, robot_id_)
{
    channel_id_      = 0;
    loop_hz_         = loop_hz;
    robot_constants_ = robot_constants;

    redis_client_ = std::make_unique<RedisClient>(REDIS_DEFAULT_HOST, REDIS_DEFAULT_PORT);

    auto robot_id   = std::stoi(redis_client_->get(ROBOT_ID_REDIS_KEY));
    auto channel_id = std::stoi(redis_client_->get(ROBOT_MULTICAST_CHANNEL_REDIS_KEY));
    auto network_interface = redis_client_->get(ROBOT_NETWORK_INTERFACE_REDIS_KEY);

    NetworkLoggerSingleton::initializeLogger(channel_id, network_interface, robot_id);

    motor_service_ = std::make_unique<MotorService>(robot_constants, loop_hz);
    power_service_ = std::make_unique<PowerService>();
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

            // Grab the latest configs from redis
            auto robot_id = std::stoi(redis_client_->get(ROBOT_ID_REDIS_KEY));
            auto channel_id =
                std::stoi(redis_client_->get(ROBOT_MULTICAST_CHANNEL_REDIS_KEY));
            auto network_interface =
                redis_client_->get(ROBOT_NETWORK_INTERFACE_REDIS_KEY);

            // If any of the configs have changed, update the network service to switch
            // to the new interface and channel with the correct robot ID
            if (robot_id != robot_id_ || channel_id != channel_id_ ||
                network_interface != network_interface_)
            {
                LOG(DEBUG) << "Switch over to Robot ID: " << robot_id
                           << " Channel ID: " << channel_id
                           << " Network Interface: " << network_interface;

                // Update the robot ID and channel ID
                robot_id_          = robot_id;
                channel_id_        = channel_id;
                network_interface_ = network_interface;

                primitive_executor_.setRobotId(robot_id_);

                network_service_ = std::make_unique<NetworkService>(
                    std::string(ROBOT_MULTICAST_CHANNELS.at(channel_id_)) + "%" +
                        network_interface_,
                    VISION_PORT, PRIMITIVE_PORT, ROBOT_STATUS_PORT, true);
            }

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

                    thunderloop_status_.set_primitive_executor_start_time_ns(
                        static_cast<unsigned long>(poll_time.tv_nsec));
                }
            }

            // If the world msg is new, update the internal buffer
            if (new_world.time_sent().epoch_timestamp_seconds() >
                world_.time_sent().epoch_timestamp_seconds())
            {
                clock_gettime(CLOCK_MONOTONIC, &last_world_recieved_time);
                primitive_executor_.updateWorld(new_world);
                world_ = new_world;
            }

            // If world not received in a while, stop robot
            struct timespec world_result;

            clock_gettime(CLOCK_MONOTONIC, &current_time);
            ScopedTimespecTimer::timespecDiff(&current_time, &last_world_recieved_time,
                                              &world_result);

            auto nanoseconds_elapsed_since_last_world =
                world_result.tv_sec * static_cast<int>(NANOSECONDS_PER_SECOND) +
                world_result.tv_nsec;

            if (nanoseconds_elapsed_since_last_world >
                static_cast<long>(WORLD_TIMEOUT_NS))
            {
                primitive_executor_.setStopPrimitive();

                // Log milliseconds since last world received if we are timing out
                LOG(WARNING) << "World timeout, overriding with StopPrimitive\n"
                             << "Milliseconds since last world: "
                             << static_cast<int>(nanoseconds_elapsed_since_last_world) *
                                    MILLISECONDS_PER_NANOSECOND;
            }

            // Primitive Executor: run the last primitive if we have not timed out
            {
                ScopedTimespecTimer timer(&poll_time);

                // Handle emergency stop override
                struct timespec primitive_result;

                clock_gettime(CLOCK_MONOTONIC, &current_time);
                ScopedTimespecTimer::timespecDiff(
                    &current_time, &last_primitive_received_time, &primitive_result);

                auto nanoseconds_elapsed_since_last_primitive =
                    primitive_result.tv_sec * static_cast<int>(NANOSECONDS_PER_SECOND) +
                    primitive_result.tv_nsec;

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
                auto kick_slope =
                    std::stoi(redis_client_->get(ROBOT_KICK_SLOPE_REDIS_KEY));
                auto kick_constant =
                    std::stoi(redis_client_->get(ROBOT_KICK_CONSTANT_REDIS_KEY));
                auto chip_pulse_width =
                    std::stoi(redis_client_->get(ROBOT_CHIP_PULSE_WIDTH_REDIS_KEY));

                power_status_ =
                    power_service_->poll(direct_control_.power_control(), kick_slope,
                                         kick_constant, chip_pulse_width);
            }
            thunderloop_status_.set_power_service_poll_time_ns(
                static_cast<unsigned long>(poll_time.tv_nsec));

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

            // Update Robot Status with poll responses
            *(robot_status_.mutable_thunderloop_status()) = thunderloop_status_;
            *(robot_status_.mutable_motor_status())       = motor_status_;
            *(robot_status_.mutable_power_status())       = power_status_;
            *(robot_status_.mutable_jetson_status())      = jetson_status_;

            // Update Redis
            redis_client_->set(ROBOT_BATTERY_VOLTAGE_REDIS_KEY,
                               std::to_string(power_status_.battery_voltage()));
            redis_client_->set(ROBOT_CURRENT_DRAW_REDIS_KEY,
                               std::to_string(power_status_.current_draw()));
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
