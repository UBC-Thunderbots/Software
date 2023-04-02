#include "software/jetson_nano/primitive_executor.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/physics/velocity_conversion_util.h"

PrimitiveExecutor::PrimitiveExecutor(const Duration time_step,
                                     const RobotConstants_t &robot_constants,
                                     const TeamColour friendly_team_colour,
                                     const RobotId robot_id)
    : current_primitive_(),
      friendly_team_colour(friendly_team_colour),
      robot_constants_(robot_constants),
      hrvo_simulator_(),
      time_step_(time_step),
      curr_orientation_(Angle::zero()),
      robot_id_(robot_id)
{
}

void PrimitiveExecutor::updatePrimitiveSet(
    const TbotsProto::PrimitiveSet &primitive_set_msg)
{
    hrvo_simulator_.updatePrimitiveSet(primitive_set_msg, time_step_);
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
    hrvo_simulator_.updateWorld(new_world, robot_constants_, time_step_);

    auto this_robot = new_world.friendlyTeam().getRobotById(robot_id_);
    if (this_robot.has_value())
    {
        curr_orientation_ = this_robot->orientation();
    }
}

void PrimitiveExecutor::updateVelocity(const Vector &local_velocity,
                                       const AngularVelocity &angular_velocity)
{
    // TODO: Add feedback logic here

//    Vector curr_velocity = hrvo_simulator_.getRobotVelocity(robot_id_);
//    if ((curr_velocity - local_velocity).length() > 3.0)
//    {
//        LOG(INFO) << "Update robot linear velocity";
//        hrvo_simulator_.updateRobotVelocity(
//            robot_id_, localToGlobalVelocity(local_velocity, curr_orientation_));
//    }
//
//    AngularVelocity curr_angular_velocity = hrvo_simulator_.getRobotAngularVelocity(robot_id_);
//    if (angular_velocity.minDiff(curr_angular_velocity).toRadians() > 10.0)
//    {
//        LOG(INFO) << "Update robot angular velocity";
//        hrvo_simulator_.updateRobotAngularVelocity(
//            robot_id_, angular_velocity);
//    }
}

Vector PrimitiveExecutor::getTargetLinearVelocity()
{
    Vector target_global_velocity = hrvo_simulator_.getRobotVelocity(robot_id_);
    return globalToLocalVelocity(target_global_velocity, curr_orientation_);
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity()
{
    return hrvo_simulator_.getRobotAngularVelocity(robot_id_);
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive()
{
    hrvo_simulator_.doStep(time_step_);

    // Visualize the HRVO Simulator for the current robot
    hrvo_simulator_.visualize(robot_id_, friendly_team_colour);

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
                    getTargetAngularVelocity();

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
