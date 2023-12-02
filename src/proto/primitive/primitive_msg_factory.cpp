#include "proto/primitive/primitive_msg_factory.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/logger/logger.h"
#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"
#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d_angular.h"

std::unique_ptr<TbotsProto::Primitive> createDirectControlPrimitive(
    const Vector &velocity, AngularVelocity angular_velocity, double dribbler_speed_rpm,
    const TbotsProto::AutoChipOrKick &auto_chip_or_kick)
{
    auto direct_control_primitive_msg = std::make_unique<TbotsProto::Primitive>();
    auto direct_velocity_control =
        std::make_unique<TbotsProto::MotorControl::DirectVelocityControl>();

    *(direct_velocity_control->mutable_velocity()) = *createVectorProto(velocity);
    *(direct_velocity_control->mutable_angular_velocity()) =
        *createAngularVelocityProto(angular_velocity);

    *(direct_control_primitive_msg->mutable_direct_control()
          ->mutable_motor_control()
          ->mutable_direct_velocity_control()) = *direct_velocity_control;

    direct_control_primitive_msg->mutable_direct_control()
        ->mutable_motor_control()
        ->set_dribbler_speed_rpm(static_cast<float>(dribbler_speed_rpm));

    *(direct_control_primitive_msg->mutable_direct_control()
          ->mutable_power_control()
          ->mutable_chicker()
          ->mutable_auto_chip_or_kick()) = auto_chip_or_kick;
    return direct_control_primitive_msg;
}
