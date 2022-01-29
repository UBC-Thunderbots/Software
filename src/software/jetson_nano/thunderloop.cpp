#include "software/jetson_nano/thunderloop.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "proto/tbots_software_msgs.pb.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/jetson_nano/services/motor.h"
#include "software/logger/logger.h"
#include "software/world/robot_state.h"

Thunderloop::Thunderloop(const RobotConstants_t& robot_constants,
                         const WheelConstants_t& wheel_consants)
{
    robot_id_        = 0;
    robot_constants_ = robot_constants;
    wheel_consants_  = wheel_consants;

    motor_service_ = std::make_unique<MotorService>(robot_constants, wheel_consants);

    // TODO (#2331) remove this once we receive actual vision data
    current_robot_state_ =
        std::make_unique<RobotState>(Point(), Vector(), Angle(), AngularVelocity());
}

Thunderloop::~Thunderloop()
{
    // De-initialize Services
    motor_service_->stop();
}

/*
 * Run the main robot loop!
 *
 * @param The rate to run the loop
 */
void Thunderloop::run(unsigned run_at_hz)
{
    using clock = std::chrono::steady_clock;

    auto next_frame = clock::now();

    motor_service_->start();

    for (;;)
    {
        // TODO (#2335) add loop timing introspection and use Preempt-RT (maybe)
        next_frame += std::chrono::milliseconds(
            static_cast<int>(MILLISECONDS_PER_SECOND / run_at_hz));

        // TODO (#2331) poll network service and update current_robot_state_
        // TODO (#2333) poll redis service

        // Execute latest primitive
        primitive_executor_.updatePrimitiveSet(robot_id_, primitive_set_);
        direct_control_ = *primitive_executor_.stepPrimitive(0, *current_robot_state_);

        // Poll motor service with wheel velocities and dribbler rpm
        // TODO (#2332) properly implement, this is just a placeholder
        drive_units_status_ =
            *motor_service_->poll(direct_control_.direct_velocity_control(),
                                  direct_control_.dribbler_speed_rpm());

        // TODO (#2334) power service poll
        std::this_thread::sleep_until(next_frame);
    }
}
