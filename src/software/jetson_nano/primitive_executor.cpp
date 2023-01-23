#include "software/jetson_nano/primitive_executor.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/physics/velocity_conversion_util.h"

PrimitiveExecutor::PrimitiveExecutor(const double time_step,
                                     const RobotConstants_t &robot_constants,
                                     const TeamColour friendly_team_colour,
                                     const RobotId robot_id)
    : current_primitive_(),
      robot_constants_(robot_constants),
      hrvo_simulator_(static_cast<float>(time_step), robot_constants,
                      friendly_team_colour),
      time_step_(time_step),
      curr_orientation_(Angle::zero()),
      robot_id_(robot_id)
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
        return;
    }
}

void PrimitiveExecutor::setStopPrimitive()
{
    current_primitive_ = *createStopPrimitive();
}

void PrimitiveExecutor::updateWorld(const TbotsProto::World &world_msg)
{
    World new_world = World(world_msg);
    hrvo_simulator_.updateWorld(new_world);

    auto this_robot = new_world.friendlyTeam().getRobotById(robot_id_);
    if (this_robot.has_value())
    {
        curr_orientation_ = this_robot->orientation();
    }
}

void PrimitiveExecutor::updateVelocity(const Vector &local_velocity,
                                       const AngularVelocity &angular_velocity)
{
    hrvo_simulator_.updateRobotVelocity(
        robot_id_, localToGlobalVelocity(local_velocity, curr_orientation_));
}

Vector PrimitiveExecutor::getTargetLinearVelocity()
{
    Vector target_global_velocity = hrvo_simulator_.getRobotVelocity(robot_id_);
    return globalToLocalVelocity(target_global_velocity, curr_orientation_);
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity(
    const TbotsProto::MovePrimitive &move_primitive)
{
    const Angle dest_orientation = createAngle(move_primitive.final_angle());
    const double delta_orientation =
        dest_orientation.minDiff(curr_orientation_).toRadians();

    // angular velocity given linear deceleration and distance remaining to target
    // orientation.
    // Vi = sqrt(0^2 + 2 * a * d)
    double deceleration_angular_speed = std::sqrt(
        2 * robot_constants_.robot_max_ang_acceleration_rad_per_s_2 * delta_orientation);

    double max_angular_speed =
        static_cast<double>(robot_constants_.robot_max_ang_speed_rad_per_s);
    double next_angular_speed = std::min(max_angular_speed, deceleration_angular_speed);

    const double signed_delta_orientation =
        (dest_orientation - curr_orientation_).clamp().toRadians();
    auto target_angular_velocity = AngularVelocity::fromRadians(
        std::copysign(next_angular_speed, signed_delta_orientation));

    // Update estimated orientation for next iteration
    curr_orientation_ += target_angular_velocity * time_step_;

    return target_angular_velocity;
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive()
{
    hrvo_simulator_.doStep();

<<<<<<< HEAD
    // Visualize the Hrvo Simulator for the current robot
    hrvo_simulator_.visualize(robot_id);
=======
    // Visualize the HRVO Simulator for the current robot
    hrvo_simulator_.visualize(robot_id_);
>>>>>>> c070809816e7711d3e1d65d4a1aef0a54d866bb7

    switch (current_primitive_.primitive_case())
    {
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
            Vector target_velocity = getTargetLinearVelocity();
            AngularVelocity target_angular_velocity =
                getTargetAngularVelocity(current_primitive_.move());

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
