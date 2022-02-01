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
    // add network service here

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
 */
void Thunderloop::runLoop()
{
    // TODO start power service
    motor_service_->start();
    // motor_service_->startEncoderCalibration();

    for (;;)
    {
        // TODO (#2331) poll network service and update current_robot_state_
        // TODO (#2333) poll redis service

        // Execute latest primitive
        primitive_executor_.startPrimitive(robot_constants_, primitive_);
        direct_control_ = *primitive_executor_.stepPrimitive(*current_robot_state_);

        // Run the motor service with the direct_control_ msg
        drive_units_status_ = *motor_service_->poll(direct_control_);
    }
}
