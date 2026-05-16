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
                                     const RobotConstants& robot_constants,
                                     const TeamColour friendly_team_colour,
                                     const RobotId robot_id)
    : friendly_team_colour_(friendly_team_colour),
      robot_constants_(robot_constants),
      time_step_(time_step),
      robot_id_(robot_id)
{
}

bool PrimitiveExecutor::isLateralTrajectoryNew(const std::optional<TrajectoryPath>& new_trajectory) const
{
    if (new_trajectory.has_value() != trajectory_path_.has_value())
    {
        // Either we don't have a trajectory right now, and this one does, or we have one and this trajectory doesn't.
        return true;
    }
    if (!new_trajectory.has_value())
    {
        return false;
    }
    return distance(new_trajectory->getDestination(), trajectory_path_->getDestination()) > LATERAL_DESTINATION_THRESHOLD_METERS;
}

bool PrimitiveExecutor::isAngularTrajectoryNew(const std::optional<BangBangTrajectory1DAngular>& new_trajectory) const
{
    if (new_trajectory.has_value() != trajectory_path_.has_value())
    {
        // Either we don't have a trajectory right now, and this one does, or we have one and this trajectory doesn't.
        return true;
    }
    if (!new_trajectory.has_value())
    {
        return false;
    }
    return new_trajectory->getDestination().minDiff(angular_trajectory_->getDestination()).toDegrees() > ANGULAR_DESTINATION_THRESHOLD_DEGREES;
}

void PrimitiveExecutor::updatePrimitive(const TbotsProto::Primitive& primitive_msg)
{
    current_primitive_ = primitive_msg;


    if (current_primitive_.has_move())
    {
        const std::optional new_trajectory_path = createTrajectoryPathFromParams(current_primitive_.move().xy_traj_params(),
                                           position_, velocity_, robot_constants_);
        // Check if this new trajectory is "new". That is, if its destination differs meaningfully from our current
        // trajectory.
        if (isLateralTrajectoryNew(new_trajectory_path))
        {
            trajectory_path_ = new_trajectory_path;
            time_since_linear_trajectory_creation_  = Duration::fromSeconds(RTT_S);
            x_pid.reset();
            y_pid.reset();
        }

        const std::optional new_angular_trajectory = createAngularTrajectoryFromParams(
            current_primitive_.move().w_traj_params(), orientation_, angular_velocity_,
            robot_constants_);

        if (isAngularTrajectoryNew(new_angular_trajectory))
        {
            angular_trajectory_ = new_angular_trajectory;
            time_since_angular_trajectory_creation_ = Duration::fromSeconds(RTT_S);
            w_pid.reset();
        }
    }
}

void PrimitiveExecutor::updateState(const Point& position, const Vector& velocity,
                                    const Angle& orientation,
                                    const AngularVelocity& angular_velocity)
{
    position_         = position;
    // velocity_         = velocity;
    orientation_      = orientation;
    // angular_velocity_ = angular_velocity;
    // If we are lagging behind trajectory too much, we have stalled! We need to regenerate trajectory.
    // TODO: Add a timeout to following error
    const double lateral_following_error = (position_ - trajectory_path_->getPosition(time_since_linear_trajectory_creation_.toSeconds())).length();
    if (lateral_following_error > LATERAL_STALL_ERROR_MAX_METERS)
    {
        // regenerate lateral trajectory
        trajectory_path_ = createTrajectoryPathFromParams(current_primitive_.move().xy_traj_params(), position_, velocity_, robot_constants_);
    }
    const double angular_following_error = angular_trajectory_->getPosition(time_since_angular_trajectory_creation_.toSeconds()).minDiff(orientation_).toDegrees();
    if (angular_following_error > ANGULAR_STALL_ERROR_MAX_DEGREES)
    {
        // regenerate angular trajectory
        angular_trajectory_ = createAngularTrajectoryFromParams(current_primitive_.move().w_traj_params(), orientation_, angular_velocity_, robot_constants_);
    }

}

Vector PrimitiveExecutor::getTargetLinearVelocity()
{
    Vector local_velocity = globalToLocalVelocity(
        trajectory_path_->getVelocity(time_since_linear_trajectory_creation_.toSeconds()),
        orientation_);
    // orientation_ + angular_velocity_ * time_step_.toSeconds() / 2 * LEAN_BIAS);

    const Point expected_position =
        trajectory_path_->getPosition(time_since_linear_trajectory_creation_.toSeconds());
    const double distance_to_destination =
        distance(expected_position, trajectory_path_->getDestination());

    // Dampen velocity as we get closer to the destination to reduce jittering
    if (distance_to_destination < MAX_DAMPENING_LINEAR_VELOCITY_DISTANCE_M)
    {
        local_velocity *=
            distance_to_destination / MAX_DAMPENING_LINEAR_VELOCITY_DISTANCE_M;
    }

    velocity_ = localToGlobalVelocity(local_velocity, orientation_);

    return local_velocity;
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity()
{
    AngularVelocity angular_velocity = angular_trajectory_->getVelocity(
        time_since_angular_trajectory_creation_.toSeconds());

    // const Angle expected_orientation = angular_trajectory_->getPosition(
    //     time_since_angular_trajectory_creation_.toSeconds());
    // const Angle error = (orientation_ - expected_orientation).clamp();
    // angular_velocity  = angular_velocity + error * ORIENTATION_KP;

    const Angle expected_orientation = angular_trajectory_->getPosition(
        time_since_angular_trajectory_creation_.toSeconds());
    const double distance_to_destination =
        expected_orientation.minDiff(angular_trajectory_->getDestination()).toDegrees();

    // Dampen velocity as we get closer to the destination to reduce jittering
    if (distance_to_destination < MAX_DAMPENING_ANGULAR_VELOCITY_DISTANCE_DEGREES)
    {
        angular_velocity *=
            distance_to_destination / MAX_DAMPENING_ANGULAR_VELOCITY_DISTANCE_DEGREES;
    }

    angular_velocity_ = angular_velocity;

    LOG(PLOTJUGGLER) << *createPlotJugglerValue({
        {"orientation", orientation_.toRadians()},
        // {"angular_velocity", angular_velocity_.toRadians()},
        {"target_angular_velocity", angular_velocity.toRadians()},
        {"expected_orientation", expected_orientation.toRadians()},
        {"acceleration",
         angular_trajectory_
             ->getAcceleration(time_since_linear_trajectory_creation_.toSeconds())
             .toRadians()},
    });

    return angular_velocity;
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    TbotsProto::PrimitiveExecutorStatus& status)
{
    time_since_angular_trajectory_creation_ += time_step_;
    time_since_linear_trajectory_creation_ += time_step_;
    status.set_running_primitive(true);

    switch (current_primitive_.primitive_case())
    {
        case TbotsProto::Primitive::kStop:
        {
            const auto prim = createDirectControlPrimitive(
                Vector(), AngularVelocity(), 0.0, TbotsProto::AutoChipOrKick());
            status.set_running_primitive(false);
            return std::make_unique<TbotsProto::DirectControlPrimitive>(
                prim->direct_control());
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
                const auto prim = createDirectControlPrimitive(
                    Vector(), AngularVelocity(), 0.0, TbotsProto::AutoChipOrKick());
                LOG(INFO)
                    << "Not moving because trajectory_path_ or angular_trajectory_ is not set";
                return std::make_unique<TbotsProto::DirectControlPrimitive>(
                    prim->direct_control());
            }

            // FEEDFORWARD velocities from the trajectory
            const Vector trajectory_linear_velocity     = getTargetLinearVelocity();
            const AngularVelocity trajectory_angular_velocity = getTargetAngularVelocity();

            const Point target_position = trajectory_path_->getDestination();
            const Angle target_orientation = angular_trajectory_->getDestination();
            const Vector error_lateral = target_position - position_;
            const Angle error_angular = (target_orientation - orientation_).clamp();
            const Vector pid_effort_lateral(x_pid.step(error_lateral.x()), y_pid.step(error_lateral.y()));
            const AngularVelocity pid_effort_angular = AngularVelocity::fromRadians(
                w_pid.step(error_angular.toRadians()));

            Vector target_linear_velocity = globalToLocalVelocity(trajectory_linear_velocity + pid_effort_lateral, orientation_);
            target_linear_velocity = target_linear_velocity.normalize(std::min(target_linear_velocity.lengthSquared(), static_cast<double>(robot_constants_.robot_max_speed_m_per_s)));
            const AngularVelocity target_angular_velocity = (trajectory_angular_velocity + pid_effort_angular);

            const auto prim = createDirectControlPrimitive(
                target_linear_velocity, target_angular_velocity,
                convertDribblerModeToDribblerSpeed(
                    current_primitive_.move().dribbler_mode(), robot_constants_),
                current_primitive_.move().auto_chip_or_kick());

            return std::make_unique<TbotsProto::DirectControlPrimitive>(
                prim->direct_control());
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
