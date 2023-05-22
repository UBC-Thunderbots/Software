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
      current_world_(),
      friendly_team_colour(friendly_team_colour),
      robot_constants_(robot_constants),
      hrvo_simulator_(robot_id),
      time_step_(time_step),
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
    // Only update HRVO simulator if the world is newer than the previous world
    if (world_msg.time_sent().epoch_timestamp_seconds() >
        current_world_.time_sent().epoch_timestamp_seconds())
    {
        hrvo_simulator_.updateWorld(World(world_msg), robot_constants_, time_step_);
    }
}

void PrimitiveExecutor::updateVelocity(const Vector &local_velocity,
                                       const AngularVelocity &angular_velocity)
{
    // To allow robots to accelerate smoothly, we only update their simulated velocity if
    // it is significantly different from the actual robot velocity

    std::optional<Angle> orientation_opt = hrvo_simulator_.getRobotOrientation(robot_id_);
    if (!orientation_opt.has_value())
    {
        return;
    }

    Vector curr_hrvo_velocity = hrvo_simulator_.getRobotVelocity(robot_id_);
    Vector actual_global_velocity =
        localToGlobalVelocity(local_velocity, orientation_opt.value());
    if ((curr_hrvo_velocity - actual_global_velocity).length() >
        LINEAR_VELOCITY_FEEDBACK_THRESHOLD_M_PER_S)
    {
        hrvo_simulator_.updateRobotVelocity(
            robot_id_, localToGlobalVelocity(local_velocity, orientation_opt.value()));
    }

    AngularVelocity curr_angular_velocity =
        hrvo_simulator_.getRobotAngularVelocity(robot_id_);
    if (angular_velocity.minDiff(curr_angular_velocity).toDegrees() >
        ANGULAR_VELOCITY_FEEDBACK_THRESHOLD_DEG_PER_S)
    {
        hrvo_simulator_.updateRobotAngularVelocity(robot_id_, angular_velocity);
    }
}

Vector PrimitiveExecutor::getTargetLinearVelocity()
{
    std::optional<Angle> orientation_opt = hrvo_simulator_.getRobotOrientation(robot_id_);
    if (orientation_opt.has_value())
    {
        Vector target_global_velocity = hrvo_simulator_.getRobotVelocity(robot_id_);
        return globalToLocalVelocity(target_global_velocity, orientation_opt.value());
    }
    else
    {
        return Vector();
    }
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
            Vector target_velocity                  = getTargetLinearVelocity();
            AngularVelocity target_angular_velocity = getTargetAngularVelocity();

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
