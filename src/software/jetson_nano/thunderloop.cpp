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

Thunderloop::Thunderloop(const RobotConstants_t& robot_constants,
                         const WheelConstants_t& wheel_consants)
{
    robot_id_        = 0;
    robot_constants_ = robot_constants;
    wheel_consants_  = wheel_consants;

    motor_service_   = std::make_unique<MotorService>(robot_constants, wheel_consants);
    network_service_ = std::make_unique<NetworkService>(
        ROBOT_MULTICAST_CHANNELS[channel_id_], VISION_PORT, PRIMITIVE_PORT,
        ROBOT_STATUS_PORT, true);
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
    motor_service_->start();

    TbotsProto::PrimitiveSet new_primitive_set;
    TbotsProto::Vision new_vision;

    for (;;)
    {
        std::tie(new_primitive_set, new_vision) = network_service_->poll(robot_status_);

        // If the primitive msg is new, update the internal buffer
        if (new_primitive_set.time_sent().epoch_timestamp_seconds() >
            primitive_set_.time_sent().epoch_timestamp_seconds())
        {
            // Cache
            primitive_set_ = new_primitive_set;

            // If we have a primitive for "this" robot, lets update it
            if (new_primitive_set.robot_primitives().count(robot_id_) != 0)
            {
                primitive_ = new_primitive_set.mutable_robot_primitives()->at(robot_id_);

                // ========= Execute new primitive ========
                primitive_executor_.startPrimitive(robot_constants_, primitive_);
            }
        }

        // If the vision msg is new, update the internal buffer
        if (new_vision.time_sent().epoch_timestamp_seconds() >
            vision_.time_sent().epoch_timestamp_seconds())
        {
            // Cache
            vision_ = new_vision;

            // If there is a detection for "this" robot, lets update it
            if (new_vision.robot_states().count(robot_id_) != 0)
            {
                robot_state_ = new_vision.mutable_robot_states()->at(robot_id_);
            }
        }

        // TODO (#2333) poll redis service
        direct_control_ =
            *primitive_executor_.stepPrimitive(createRobotStateFromProto(robot_state_));

        // Run the motor service with the direct_control_ msg
        drive_units_status_ = *motor_service_->poll(direct_control_);
    }
}
