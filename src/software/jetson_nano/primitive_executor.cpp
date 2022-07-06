#include "software/jetson_nano/primitive_executor.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/math/math_functions.h"
#include "proto/message_translation/tbots_protobuf.h"

PrimitiveExecutor::PrimitiveExecutor(const double time_step, const RobotId robot_id,
                                     const RobotConstants_t &robot_constants,
                                     const TeamColour friendly_team_colour)
    : robot_id_(robot_id),
      robot_constants_(robot_constants),
      current_primitive_(),
      hrvo_simulator_(static_cast<float>(time_step), robot_constants,
                      friendly_team_colour),
      team_colour_testing(friendly_team_colour)
{
}

void PrimitiveExecutor::updatePrimitiveSet(
    const TbotsProto::PrimitiveSet &primitive_set_msg)
{
    hrvo_simulator_.updatePrimitiveSet(primitive_set_msg);
    auto primitive_set_msg_iter = primitive_set_msg.robot_primitives().find(robot_id_);
    if (primitive_set_msg_iter != primitive_set_msg.robot_primitives().end())
    {
        current_primitive_ = primitive_set_msg_iter->second;
    }
}

void PrimitiveExecutor::updateWorld(const TbotsProto::World &world_msg)
{
    //    if (robot_id_ == 3 && team_colour_testing == TeamColour::BLUE)
    //        std::cout << "velocity updated from world" << std::endl;
    hrvo_simulator_.updateWorld(World(world_msg));
}

void PrimitiveExecutor::updateLocalVelocity(const Vector &local_velocity,
                                            const Angle &curr_orientation)
{
//    Vector prev_vel = getTargetLinearVelocity(curr_orientation);
//    if (robot_id_ == 3 && team_colour_testing == TeamColour::BLUE)
//        std::cout << "velocity updated from sim (actual): " << local_velocity.length() <<  << std::endl;

    hrvo_simulator_.updateFriendlyRobotVelocity(robot_id_,
                                                local_velocity.rotate(-curr_orientation));
}

Vector PrimitiveExecutor::getTargetLinearVelocity(const Angle &curr_orientation)
{
    Vector target_global_velocity = hrvo_simulator_.getRobotVelocity(robot_id_);
    return target_global_velocity.rotate(-curr_orientation);
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity(
    const TbotsProto::MovePrimitive &move_primitive, const Angle &curr_orientation)
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
    const Angle &curr_orientation)
{
    Vector target_velocity2 = getTargetLinearVelocity(curr_orientation);
    if (robot_id_ == 3 && team_colour_testing == TeamColour::BLUE)
        std::cout << "hrvo velocity before doStep: " << target_velocity2.length() << std::endl;

    hrvo_simulator_.doStep();

    // Visualize the HRVO Simulator for the current robot
    hrvo_simulator_.visualize(robot_id_);

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
            output->mutable_power_control()->set_charge_mode(
                TbotsProto::PowerControl_ChargeMode_DISCHARGE);

            return output;
        }
        case TbotsProto::Primitive::kStop:
        {
            auto prim   = createDirectControlPrimitive(Vector(), AngularVelocity(), 0.0,
                                                     TbotsProto::AutoChipOrKick());
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
            Vector target_velocity = getTargetLinearVelocity(curr_orientation);
            if (robot_id_ == 4 && team_colour_testing == TeamColour::BLUE)
            {
                auto accel = (target_velocity2 - target_velocity).length() * 60.0;
                LOG(VISUALIZE) << *createNamedValue(
                            "hrvo accel",
                            static_cast<float>(accel));
                LOG(VISUALIZE) << *createNamedValue(
                            "hrvo vel",
                            static_cast<float>(target_velocity.length()));
            }

            AngularVelocity target_angular_velocity =
                getTargetAngularVelocity(current_primitive_.move(), curr_orientation);

            auto output = createDirectControlPrimitive(
                target_velocity, target_angular_velocity,
                current_primitive_.move().dribbler_speed_rpm(),
                current_primitive_.move().auto_chip_or_kick());

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

void PrimitiveExecutor::setRobotId(const RobotId robot_id)
{
    robot_id_ = robot_id;
}
