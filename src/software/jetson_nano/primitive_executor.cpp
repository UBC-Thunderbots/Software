#include "software/jetson_nano/primitive_executor.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/logger/logger.h"
#include "software/math/math_functions.h"

PrimitiveExecutor::PrimitiveExecutor(const double time_step,
                                     const RobotConstants_t& robot_constants)
    : current_primitive_(),
      robot_constants_(robot_constants),
      hrvo_simulator_(static_cast<float>(time_step), robot_constants)
{
}

void PrimitiveExecutor::updatePrimitiveSet(
    const unsigned int robot_id, const TbotsProto::PrimitiveSet& primitive_set_msg)
{
    hrvo_simulator_.updatePrimitiveSet(primitive_set_msg);
    auto primitive_set_msg_iter = primitive_set_msg.robot_primitives().find(robot_id);
    if (primitive_set_msg_iter != primitive_set_msg.robot_primitives().end())
    {
        current_primitive_ = primitive_set_msg_iter->second;
    }
}

void PrimitiveExecutor::updateWorld(const TbotsProto::World& world_msg)
{
    hrvo_simulator_.updateWorld(World(world_msg));
}

Vector PrimitiveExecutor::getTargetLinearVelocity(const unsigned int robot_id,
                                                  const Angle& curr_orientation)
{
    Vector target_global_velocity = hrvo_simulator_.getRobotVelocity(robot_id);

    double local_x_velocity = curr_orientation.cos() * target_global_velocity.x() +
                              curr_orientation.sin() * target_global_velocity.y();

    double local_y_velocity = -curr_orientation.sin() * target_global_velocity.x() +
                              curr_orientation.cos() * target_global_velocity.y();

    return Vector(local_x_velocity, local_y_velocity)
        .normalize(target_global_velocity.length());
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity(
    const TbotsProto::MovePrimitive& move_primitive, const Angle& curr_orientation)
{
    const Angle dest_orientation = createAngle(move_primitive.final_angle());
    const double delta_orientation =
        dest_orientation.minDiff(curr_orientation).toRadians();

    // angular velocity given linear deceleration and distance remaining to target
    // orientation.
    // Vi = sqrt(0^2 + 2 * a * d)
    double deceleration_angular_speed = std::sqrt(
        2 * robot_constants_.robot_max_ang_acceleration_rad_per_s_2 * delta_orientation);

    double max_angular_speed =
        static_cast<double>(robot_constants_.robot_max_ang_speed_rad_per_s);
    double next_angular_speed = std::min(max_angular_speed, deceleration_angular_speed);

    const double signed_delta_orientation =
        (dest_orientation - curr_orientation).clamp().toRadians();
    return AngularVelocity::fromRadians(
        std::copysign(next_angular_speed, signed_delta_orientation));
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    const unsigned int robot_id, const Angle& curr_orientation)
{
    hrvo_simulator_.doStep();
    // TODO (#2499): Remove if and visualize the HRVO Simulator of all robots
    if (robot_id == 1)
    {
        // All robots should have identical HRVO simulations. To avoid spam, only
        // the HRVO simulation for robot 0 will be sent to Thunderscope.
        hrvo_simulator_.visualize(robot_id);
    }

    switch (current_primitive_.primitive_case())
    {
        case TbotsProto::Primitive::kEstop:
        {
            // Protobuf guarantees that the default values in a proto are all zero (bools
            // are false)
            //
            // https://developers.google.com/protocol-buffers/docs/proto3#default
            auto output = std::make_unique<TbotsProto::DirectControlPrimitive>();

            // Discharge the capacitors
            output->set_charge_mode(
                TbotsProto::DirectControlPrimitive_ChargeMode_DISCHARGE);

            return output;
        }
        case TbotsProto::Primitive::kStop:
        {
            auto prim   = createDirectControlPrimitive(Vector(), AngularVelocity(), 0.0);
            auto output = std::make_unique<TbotsProto::DirectControlPrimitive>(
                prim->direct_control());
            return output;
        }
        case TbotsProto::Primitive::kDirectControl:
        {
            return std::make_unique<TbotsProto::DirectControlPrimitive>(
                current_primitive_.direct_control());
        }
        case TbotsProto::Primitive::kMove:
        {
            // Compute the target velocities
            Vector target_velocity = getTargetLinearVelocity(robot_id, curr_orientation);
            AngularVelocity target_angular_velocity =
                getTargetAngularVelocity(current_primitive_.move(), curr_orientation);

            auto output = createDirectControlPrimitive(
                target_velocity, target_angular_velocity,
                current_primitive_.move().dribbler_speed_rpm());

            // Copy the AutoKickOrChip settings over
            copyAutoChipOrKick(current_primitive_.move(),
                               output->mutable_direct_control());

            return std::make_unique<TbotsProto::DirectControlPrimitive>(
                output->direct_control());
        }
        case TbotsProto::Primitive::PRIMITIVE_NOT_SET:
        {
            // TODO (#2283) Once we can add/remove robots, this log should
            // be re-enabled. Right now it just gets spammed because we command
            // 6 robots for Div B when there are 11 on the field.
            //
            // LOG(DEBUG) << "No primitive set!";
        }
    }
    return std::make_unique<TbotsProto::DirectControlPrimitive>();
}

void PrimitiveExecutor::copyAutoChipOrKick(const TbotsProto::MovePrimitive& src,
                                           TbotsProto::DirectControlPrimitive* dest)
{
    switch (src.auto_chip_or_kick().auto_chip_or_kick_case())
    {
        case TbotsProto::AutoChipOrKick::AutoChipOrKickCase::kAutokickSpeedMPerS:
        {
            dest->set_autokick_speed_m_per_s(
                src.auto_chip_or_kick().autokick_speed_m_per_s());

            break;
        }
        case TbotsProto::AutoChipOrKick::AutoChipOrKickCase::kAutochipDistanceMeters:
        {
            dest->set_autochip_distance_meters(
                src.auto_chip_or_kick().autochip_distance_meters());
            break;
        }
        case TbotsProto::AutoChipOrKick::AutoChipOrKickCase::AUTO_CHIP_OR_KICK_NOT_SET:
        {
            dest->clear_chick_command();
            break;
        }
    }
}
