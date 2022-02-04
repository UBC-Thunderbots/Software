#include "software/jetson_nano/thunderloop.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/jetson_nano/services/motor.h"
#include "software/logger/logger.h"
#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"
#include "software/world/robot_state.h"

/**
 * https://rt.wiki.kernel.org/index.php/Squarewave-example
 * using clock_nanosleep of librt
 */
extern int clock_nanosleep(clockid_t __clock_id, int __flags,
                           __const struct timespec* __req, struct timespec* __rem);

Thunderloop::Thunderloop(const RobotConstants_t& robot_constants,
                         const WheelConstants_t& wheel_consants, const int loop_hz)
{
    robot_id_        = 0;
    channel_id_      = 0;
    loop_hz_         = loop_hz;
    robot_constants_ = robot_constants;
    wheel_consants_  = wheel_consants;

    motor_service_   = std::make_unique<MotorService>(robot_constants, wheel_consants);
    network_service_ = std::make_unique<NetworkService>(
        std::string(ROBOT_MULTICAST_CHANNELS[channel_id_]) + "%" + "eth0", VISION_PORT,
        PRIMITIVE_PORT, ROBOT_STATUS_PORT, true);
}

Thunderloop::~Thunderloop()
{
    // De-initialize Services
    motor_service_->stop();
}

/*
 * Run the main robot loop!
 */
void Thunderloop::runLoop()
{
    // Timing
    struct timespec next_shot;
    struct timespec time;

    // Input buffer
    TbotsProto::PrimitiveSet new_primitive_set;
    TbotsProto::Vision new_vision;

    // Loop interval
    int interval =
        static_cast<int>(1.0f / static_cast<float>(loop_hz_) * NANOSECONDS_PER_SECOND);

    // Start the services
    motor_service_->start();

    // Get current time
    // Note: CLOCK_MONOTONIC is used over CLOCK_REALTIME since CLOCK_REALTIME can jump
    // backwards
    clock_gettime(CLOCK_MONOTONIC, &next_shot);

    // Start after one second
    next_shot.tv_sec++;

    for (;;)
    {
        // Wait until next shot
        // Note: CLOCK_MONOTONIC is used over CLOCK_REALTIME since CLOCK_REALTIME can jump
        // backwards
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_shot, NULL);

        // Poll network service and grab most recent messages
        auto [new_primitive_set, new_vision] = network_service_->poll(robot_status_);
        thunderloop_status_.set_network_service_poll_time_ns(
            static_cast<unsigned long>(time.tv_nsec));

        // If the primitive msg is new, update the internal buffer
        // and start the new primitive.
        if (new_primitive_set.time_sent().epoch_timestamp_seconds() >
            primitive_set_.time_sent().epoch_timestamp_seconds())
        {
            // Save new primitive set
            primitive_set_ = new_primitive_set;

            // If we have a primitive for "this" robot, lets start it
            if (new_primitive_set.robot_primitives().count(robot_id_) != 0)
            {
                primitive_ = new_primitive_set.mutable_robot_primitives()->at(robot_id_);

                // Start new primitive
                {
                    ScopedTimespecTimer timer(&time);
                    primitive_executor_.startPrimitive(robot_constants_, primitive_);
                }

                thunderloop_status_.set_primitive_executor_start_time_ns(
                    static_cast<unsigned long>(time.tv_nsec));
            }
        }

        // If the vision msg is new, update the internal buffer
        if (new_vision.time_sent().epoch_timestamp_seconds() >
            vision_.time_sent().epoch_timestamp_seconds())
        {
            // Cache vision
            vision_ = new_vision;

            // If there is a detection for "this" robot, lets update it
            if (new_vision.robot_states().count(robot_id_) != 0)
            {
                robot_state_ = new_vision.mutable_robot_states()->at(robot_id_);
            }
        }

        // TODO (#2333) poll redis service

        {
            ScopedTimespecTimer timer(&time);
            direct_control_ =
                *primitive_executor_.stepPrimitive(createRobotState(robot_state_));
        }
        thunderloop_status_.set_primitive_executor_step_time_ns(
            static_cast<unsigned long>(time.tv_nsec));

        // Run the motor service with the direct_control_ msg
        {
            ScopedTimespecTimer timer(&time);
            drive_units_status_ = *motor_service_->poll(direct_control_);
        }
        thunderloop_status_.set_motor_service_poll_time_ns(
            static_cast<unsigned long>(time.tv_nsec));

        *(robot_status_.mutable_thunderloop_status()) = thunderloop_status_;

        robot_status_.mutable_power_status()->set_capacitor_voltage(200);

        // Calculate next shot
        next_shot.tv_nsec += interval;
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
