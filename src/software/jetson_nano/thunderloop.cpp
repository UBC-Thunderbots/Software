#include "software/jetson_nano/thunderloop.h"

#include <limits.h>
#include <malloc.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>      // Needed for mlockall()
#include <sys/resource.h>  // needed for getrusage
#include <sys/time.h>      // needed for getrusage
#include <unistd.h>        // needed for sysconf(int name);

#include <chrono>
#include <iostream>
#include <thread>

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/jetson_nano/services/motor.h"
#include "software/logger/logger.h"
#include "software/world/robot_state.h"

/**
 * https://rt.wiki.kernel.org/index.php/Squarewave-example
 * using clock_nanosleep of librt
 */
extern int clock_nanosleep(clockid_t __clock_id, int __flags,
                           __const struct timespec* __req, struct timespec* __rem);

static inline void timespecNorm(struct timespec& ts)
{
    while (ts.tv_nsec >= static_cast<int>(NANOSECONDS_PER_SECOND))
    {
        ts.tv_nsec -= static_cast<int>(NANOSECONDS_PER_SECOND);
        ts.tv_sec++;
    }
}

static inline void timespecDiff(struct timespec* a, struct timespec* b,
                                struct timespec* result)
{
    result->tv_sec  = a->tv_sec - b->tv_sec;
    result->tv_nsec = a->tv_nsec - b->tv_nsec;
    if (result->tv_nsec < 0)
    {
        --result->tv_sec;
        result->tv_nsec += 1000000000L;
    }
}


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
    struct timespec start_time;
    struct timespec end_time;
    struct timespec time;

    /**
     * Times an arbitrary expression and stores the result in time.
     *
     * @param arg The expression to time
     */
#define TIME_EXPRESSION(arg)                                                             \
    clock_gettime(0, &start_time);                                                       \
    arg;                                                                                 \
    clock_gettime(0, &end_time);                                                         \
    timespec_diff(&end_time, &start_time, &time);

    // Input buffer
    TbotsProto::PrimitiveSet new_primitive_set;
    TbotsProto::Vision new_vision;

    // Loop interval
    int interval = (1.0f / loop_hz_) * static_cast<int>(NANOSECONDS_PER_SECOND);

    // Start the services
    motor_service_->start();

    // Get current time
    clock_gettime(0, &next_shot);

    // Start after one second
    next_shot.tv_sec++;

    for (;;)
    {
        // Wait until next shot
        clock_nanosleep(0, TIMER_ABSTIME, &next_shot, NULL);

        // Poll network service and grab most recent messages
        TIME_EXPRESSION(std::tie(new_primitive_set, new_vision) =
                            network_service_->poll(robot_status_));
        thunderloop_status_.set_network_service_poll_time_ns(time.tv_nsec);

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
                TIME_EXPRESSION(
                    primitive_executor_.startPrimitive(robot_constants_, primitive_));

                thunderloop_status_.set_primitive_executor_start_time_ns(time.tv_nsec);
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

        TIME_EXPRESSION(direct_control_ = *primitive_executor_.stepPrimitive(
                            createRobotState(robot_state_)));
        thunderloop_status_.set_primitive_executor_step_time_ns(time.tv_nsec);

        // Run the motor service with the direct_control_ msg
        TIME_EXPRESSION(drive_units_status_ = *motor_service_->poll(direct_control_));
        thunderloop_status_.set_motor_service_poll_time_ns(time.tv_nsec);

        *(robot_status_.mutable_thunderloop_status()) = thunderloop_status_;

        robot_status_.mutable_power_status()->set_capacitor_voltage(200);

        // Calculate next shot
        next_shot.tv_nsec += interval;
        timespecNorm(next_shot);
    }
}
