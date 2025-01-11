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

        if (current_primitive_.has_move())
        {
            trajectory_path_ = createTrajectoryPathFromParams(
                current_primitive_.move().xy_traj_params(), velocity_, robot_constants_);

            angular_trajectory_ = createAngularTrajectoryFromParams(
                current_primitive_.move().w_traj_params(), orientation_, angular_velocity_,
                robot_constants_);

            time_since_linear_trajectory_creation_ =
                Duration::fromSeconds(VISION_TO_ROBOT_DELAY_S);
            time_since_angular_trajectory_creation_ =
                    Duration::fromSeconds(0);
        }
    }
}

void PrimitiveExecutor::setStopPrimitive()
{
    current_primitive_ = *createStopPrimitiveProto();
}

void PrimitiveExecutor::updateVelocity(const Vector &local_velocity,
                                       const AngularVelocity &angular_velocity,
                                       const Angle &orientation)
{
    orientation_ = orientation.clamp();
    Vector actual_global_velocity = localToGlobalVelocity(local_velocity, orientation.clamp());
    velocity_                     = actual_global_velocity;
    angular_velocity_             = angular_velocity;
}

Vector PrimitiveExecutor::getTargetLinearVelocity()
{
    Vector local_velocity = globalToLocalVelocity(
        trajectory_path_->getVelocity(time_since_linear_trajectory_creation_.toSeconds()),
        orientation_ + angular_velocity_ * time_step_.toSeconds() / 2 * 2);
    Point position =
        trajectory_path_->getPosition(time_since_linear_trajectory_creation_.toSeconds());
    double distance_to_destination =
        distance(position, trajectory_path_->getDestination());

    // Dampen velocity as we get closer to the destination to reduce jittering
    if (distance_to_destination < MAX_DAMPENING_VELOCITY_DISTANCE_M)
    {
        local_velocity *= distance_to_destination / MAX_DAMPENING_VELOCITY_DISTANCE_M;
    }
    return local_velocity;
}
AngularVelocity PrimitiveExecutor::getTargetAngularAcceleration() {
    if (angular_trajectory_.has_value()) {
        return angular_trajectory_->getAcceleration(time_since_angular_trajectory_creation_.toSeconds());
    } else {
        return Angle::zero();
    }
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity()
{
    AngularVelocity angular_velocity =
        angular_trajectory_->getVelocity(time_since_angular_trajectory_creation_.toSeconds());
    Angle orientation_to_destination =
        orientation_.minDiff(angular_trajectory_->getDestination());
//    LOG(INFO) << "Raw " << (angular_trajectory_->getPosition(time_since_angular_trajectory_creation_.toSeconds()) - orientation_).toDegrees();
    Angle error = orientation_.minSignedDiff(angular_trajectory_->getPosition(time_since_angular_trajectory_creation_.toSeconds()));
//    LOG(INFO) << error.toDegrees();
    angular_velocity = angular_velocity + error * 0; // TODO: MOVE kP to a constant (per robot???)
    if (orientation_to_destination.toDegrees() < 5)
    {
        angular_velocity *= orientation_to_destination.toDegrees() / 5;
    }

    return angular_velocity;
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    TbotsProto::PrimitiveExecutorStatus &status)
{
    time_since_angular_trajectory_creation_ += time_step_;
    time_since_linear_trajectory_creation_ += time_step_;
    status.set_running_primitive(true);

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

            Angle error = prev_target_angular_velocity_ - angular_velocity_;

            Vector local_velocity            = getTargetLinearVelocity();
            AngularVelocity target_angular_velocity = getTargetAngularVelocity();
//            LOG(INFO) << target_angular_velocity.toRadians();
            // add integration term
            prev_target_angular_velocity_ = target_angular_velocity;

            auto output = createDirectControlPrimitive(
                local_velocity, target_angular_velocity + error * 0.0,
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
