#include "software/embedded/primitive_executor.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"
#include "software/physics/velocity_conversion_util.h"

PrimitiveExecutor::PrimitiveExecutor(const Duration time_step,
                                     const RobotConstants_t &robot_constants,
                                     const TeamColour friendly_team_colour,
                                     const RobotId robot_id)
    : current_primitive_(),
      time_since_trajectory_creation_(Duration::fromSeconds(0)),
      friendly_team_colour_(friendly_team_colour),
      robot_constants_(robot_constants),
      time_step_(time_step),
      robot_id_(robot_id)

{
}

void PrimitiveExecutor::updatePrimitiveSet(
    const TbotsProto::PrimitiveSet &primitive_set_msg)
{
    auto primitive_set_msg_iter = primitive_set_msg.robot_primitives().find(robot_id_);
    if (primitive_set_msg_iter != primitive_set_msg.robot_primitives().end())
    {
        current_primitive_ = primitive_set_msg_iter->second;
        curr_robot_position_ = createPoint(current_primitive_.move().xy_traj_params().start_position());

        if (current_primitive_.has_move())
        {
            // TODO (NIMA):
//            Vector initial_velocity = velocity_;
//            if (trajectory_path_.has_value() && (trajectory_path_->getVelocity(time_since_trajectory_creation_.toSeconds()) - velocity_).length() > 2.0)
//            {
//                std::cout << "Initial velocity too different from reality!" << std::endl;
//                initial_velocity = trajectory_path_->getVelocity(time_since_trajectory_creation_.toSeconds()) - velocity_;
//            }

//            trajectory_path_ = createTrajectoryPathFromParams(
//                    current_primitive_.move().xy_traj_params(), initial_velocity, robot_constants_);

            angular_trajectory_ = createAngularTrajectoryFromParams(
                    current_primitive_.move().w_traj_params(), angular_velocity_,
                    robot_constants_, current_primitive_.move().max_speed_mode());

            time_since_trajectory_creation_ =
                    Duration::fromSeconds(VISION_TO_ROBOT_DELAY_S);
        }
    }
}

void PrimitiveExecutor::setStopPrimitive()
{
    current_primitive_ = *createStopPrimitiveProto();
}

void PrimitiveExecutor::updateVelocity(const Vector &local_velocity,
                                       const AngularVelocity &angular_velocity)
{
    Vector actual_global_velocity = localToGlobalVelocity(local_velocity, orientation_);
    velocity_                     = actual_global_velocity;
    angular_velocity_             = angular_velocity;
}

Vector PrimitiveExecutor::getTargetLinearVelocity()
{
    Vector local_velocity = globalToLocalVelocity(
        trajectory_path_->getVelocity(VISION_TO_ROBOT_DELAY_S),
        orientation_);
    Point position =
        trajectory_path_->getPosition(VISION_TO_ROBOT_DELAY_S);
    double distance_to_destination =
        distance(position, trajectory_path_->getDestination());

    // Dampen velocity as we get closer to the destination to reduce jittering
    if (distance_to_destination < MAX_DAMPENING_VELOCITY_DISTANCE_M)
    {
        local_velocity *= distance_to_destination / MAX_DAMPENING_VELOCITY_DISTANCE_M;
    }
    return local_velocity;
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity()
{
    orientation_ =
        angular_trajectory_->getPosition(time_since_trajectory_creation_.toSeconds());

    AngularVelocity angular_velocity =
        angular_trajectory_->getVelocity(time_since_trajectory_creation_.toSeconds());
    Angle orientation_to_destination =
        orientation_.minDiff(angular_trajectory_->getDestination());
    if (orientation_to_destination.toDegrees() < 5)
    {
        angular_velocity *= orientation_to_destination.toDegrees() / 5;
    }

    return angular_velocity;
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    TbotsProto::PrimitiveExecutorStatus &status)
{
    curr_robot_position_ = curr_robot_position_ + (velocity_ * time_step_.toSeconds());
    time_since_trajectory_creation_ += time_step_;
    status.set_running_primitive(true);


    trajectory_path_ = createTrajectoryPathFromParams(
            current_primitive_.move().xy_traj_params(), curr_robot_position_, velocity_, robot_constants_, current_primitive_.move().max_speed_mode());

    switch (current_primitive_.primitive_case())
    {
        case TbotsProto::Primitive::kStop:
        {
            auto prim   = createDirectControlPrimitive(Vector(), AngularVelocity(), 0.0,
                                                     TbotsProto::AutoChipOrKick());
            auto output = std::make_unique<TbotsProto::DirectControlPrimitive>(
                prim->direct_control());
            status.set_running_primitive(false);
            return output;
        }
        case TbotsProto::Primitive::kDirectControl:
        {
            return std::make_unique<TbotsProto::DirectControlPrimitive>(
                current_primitive_.direct_control());
        }
        case TbotsProto::Primitive::kMove:
        {
            if (!trajectory_path_.has_value() || !angular_trajectory_.has_value())
            {
                auto prim = createDirectControlPrimitive(Vector(), AngularVelocity(), 0.0,
                                                         TbotsProto::AutoChipOrKick());
                auto output = std::make_unique<TbotsProto::DirectControlPrimitive>(
                    prim->direct_control());
                LOG(INFO)
                    << "Not moving because trajectory_path_ or angular_trajectory_ is not set";
                return output;
            }

            Vector local_velocity            = getTargetLinearVelocity();
            AngularVelocity angular_velocity = getTargetAngularVelocity();

            auto output = createDirectControlPrimitive(
                local_velocity, angular_velocity,
                convertDribblerModeToDribblerSpeed(
                    current_primitive_.move().dribbler_mode(), robot_constants_),
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
