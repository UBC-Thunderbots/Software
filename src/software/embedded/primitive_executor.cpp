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

PrimitiveExecutor::PrimitiveExecutor(
    const robot_constants::RobotConstants& robot_constants)
    : robot_constants_(robot_constants)
{
}

bool PrimitiveExecutor::isLinearTrajectoryNew(
    const std::optional<TrajectoryPath>& new_trajectory) const
{
    if (new_trajectory.has_value() != trajectory_path_.has_value())
    {
        // Either we don't have a trajectory right now, and this one does, or we have one
        // and this trajectory doesn't.
        return true;
    }

    if (!new_trajectory.has_value())
    {
        return false;
    }

    return distance(new_trajectory->getDestination(),
                    trajectory_path_->getDestination()) >
           LINEAR_DESTINATION_THRESHOLD_METERS;
}

bool PrimitiveExecutor::isAngularTrajectoryNew(
    const BangBangTrajectory1DAngular& new_trajectory) const
{
    if (!angular_trajectory_.has_value())
    {
        return true;
    }

    return new_trajectory.getDestination()
               .minDiff(angular_trajectory_->getDestination())
               .toDegrees() > ANGULAR_DESTINATION_THRESHOLD_DEGREES;
}

void PrimitiveExecutor::updatePrimitive(const TbotsProto::Primitive& primitive_msg)
{
    current_primitive_ = primitive_msg;

    if (current_primitive_.has_move())
    {
        const std::optional new_trajectory_path =
            createTrajectoryPathFromParams(current_primitive_.move().xy_traj_params(),
                                           position_, velocity_, robot_constants_);

        if (isLinearTrajectoryNew(new_trajectory_path))
        {
            trajectory_path_                 = new_trajectory_path;
            time_since_linear_trajectory_creation_ =
                std::chrono::duration<double>::zero();
            x_pid.reset();
            y_pid.reset();
        }

        const BangBangTrajectory1DAngular new_angular_trajectory =
            createAngularTrajectoryFromParams(current_primitive_.move().w_traj_params(),
                                              orientation_, angular_velocity_,
                                              robot_constants_);

        if (isAngularTrajectoryNew(new_angular_trajectory))
        {
            angular_trajectory_               = new_angular_trajectory;
            time_since_angular_trajectory_creation_ =
                std::chrono::duration<double>::zero();
            w_pid.reset();
        }
    }
}

void PrimitiveExecutor::updateState(const Point& position, const Vector& velocity,
                                    const Angle& orientation,
                                    const AngularVelocity& angular_velocity)
{
    position_         = position;
    velocity_         = velocity;
    orientation_      = orientation;
    angular_velocity_ = angular_velocity;

    // If we are lagging behind trajectory too much, we have stalled! We need to
    // regenerate trajectory.
    // TODO: Add a timeout to following error
    if (trajectory_path_.has_value())
    {
        const double linear_following_error =
            (position_ - trajectory_path_->getPosition(
                             time_since_linear_trajectory_creation_.count()))
                .length();

        if (linear_following_error > LINEAR_STALL_ERROR_MAX_METERS)
        {
            // regenerate linear trajectory
            trajectory_path_ =
                createTrajectoryPathFromParams(current_primitive_.move().xy_traj_params(),
                                               position_, velocity_, robot_constants_);

            time_since_linear_trajectory_creation_ =
                std::chrono::duration<double>::zero();
        }
    }

    if (angular_trajectory_.has_value())
    {
        const double angular_following_error =
            angular_trajectory_
                ->getPosition(time_since_angular_trajectory_creation_.count())
                .minDiff(orientation_)
                .toDegrees();

        if (angular_following_error > ANGULAR_STALL_ERROR_MAX_DEGREES)
        {
            // regenerate angular trajectory
            angular_trajectory_ = createAngularTrajectoryFromParams(
                current_primitive_.move().w_traj_params(), orientation_,
                angular_velocity_, robot_constants_);

            time_since_angular_trajectory_creation_ =
                std::chrono::duration<double>::zero();
        }
    }
}

Vector PrimitiveExecutor::getTargetLinearVelocity() const
{
    Vector target_velocity =
        trajectory_path_->getVelocity(time_since_linear_trajectory_creation_.count());

    const Point expected_position =
        trajectory_path_->getPosition(time_since_linear_trajectory_creation_.count());
    const double distance_to_destination =
        distance(expected_position, trajectory_path_->getDestination());

    // Dampen velocity as we get closer to the destination to reduce jittering
    if (distance_to_destination < MAX_DAMPENING_LINEAR_VELOCITY_DISTANCE_M)
    {
        target_velocity *=
            distance_to_destination / MAX_DAMPENING_LINEAR_VELOCITY_DISTANCE_M;
    }

    return target_velocity;
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity() const
{
    AngularVelocity angular_velocity =
        angular_trajectory_->getVelocity(time_since_angular_trajectory_creation_.count());

    const Angle expected_orientation =
        angular_trajectory_->getPosition(time_since_angular_trajectory_creation_.count());
    const double distance_to_destination =
        expected_orientation.minDiff(angular_trajectory_->getDestination()).toDegrees();

    // Dampen velocity as we get closer to the destination to reduce jittering
    if (distance_to_destination < MAX_DAMPENING_ANGULAR_VELOCITY_DISTANCE_DEGREES)
    {
        angular_velocity *=
            distance_to_destination / MAX_DAMPENING_ANGULAR_VELOCITY_DISTANCE_DEGREES;
    }

    return angular_velocity;
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    TbotsProto::PrimitiveExecutorStatus& status)
{
    const auto current_time = std::chrono::steady_clock::now();
    time_since_linear_trajectory_creation_ += current_time - last_step_time_;
    time_since_angular_trajectory_creation_ += current_time - last_step_time_;
    last_step_time_ = current_time;

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
                LOG(INFO) << "Not moving because trajectory_path_ has value "
                          << trajectory_path_.has_value() << " angular has "
                          << angular_trajectory_.has_value();
                return std::make_unique<TbotsProto::DirectControlPrimitive>(
                    prim->direct_control());
            }

            const Point target_position    = trajectory_path_->getDestination();
            const Angle target_orientation = angular_trajectory_->getDestination();
            Vector error_linear            = target_position - position_;
            Angle error_angular            = (target_orientation - orientation_).clamp();

            Vector target_linear_velocity;
            AngularVelocity target_angular_velocity;

            // if close enough, use special PID to destination
            if (error_linear.lengthSquared() < LINEAR_PURE_PID_THRESHOLD_METERS)
            {
                target_linear_velocity =
                    globalToLocalVelocity({x_pid_close.step(error_linear.x()),
                                           y_pid_close.step(error_linear.y())},
                                          orientation_);
            }
            else
            {
                // FEEDFORWARD velocities from the trajectory
                const Vector trajectory_linear_velocity = getTargetLinearVelocity();
                error_linear                            = trajectory_path_->getPosition(
                                                              time_since_linear_trajectory_creation_.count()) -
                               position_;
                const Vector pid_effort_linear(x_pid.step(error_linear.x()),
                                               y_pid.step(error_linear.y()));
                target_linear_velocity = globalToLocalVelocity(
                    trajectory_linear_velocity + pid_effort_linear, orientation_);
            }

            if (error_angular.abs().toDegrees() < ANGULAR_PURE_PID_THRESHOLD_DEGREES)
            {
                target_angular_velocity = AngularVelocity::fromRadians(
                    w_pid_close.step(error_angular.toRadians()));

                LOG(PLOTJUGGLER) << *createPlotJugglerValue({
                    {"is_pure_pid", 100},
                });
            }
            else
            {
                // FEEDFORWARD velocities from the trajectory
                error_angular = (angular_trajectory_->getPosition(
                                     time_since_angular_trajectory_creation_.count()) -
                                 orientation_)
                                    .clamp();
                const AngularVelocity trajectory_angular_velocity =
                    getTargetAngularVelocity();
                const AngularVelocity pid_effort_angular =
                    AngularVelocity::fromRadians(w_pid.step(error_angular.toRadians()));
                target_angular_velocity =
                    trajectory_angular_velocity + pid_effort_angular;

                LOG(PLOTJUGGLER) << *createPlotJugglerValue({
                    {"is_pure_pid", 0},
                });
            }

            LOG(PLOTJUGGLER) << *createPlotJugglerValue({
                {"target_angular_velocity", target_angular_velocity.toDegrees()},
                {"angular_velocity", angular_velocity_.toDegrees()},
                {"orientation", orientation_.toDegrees()},
                {"expected_orientation",
                 angular_trajectory_
                     ->getPosition(time_since_angular_trajectory_creation_.count())
                     .toDegrees()},
            });

            // Make sure target linear velocity is clamped
            target_linear_velocity = target_linear_velocity.normalize(
                std::min(target_linear_velocity.length(),
                         static_cast<double>(robot_constants_.robot_max_speed_m_per_s)));

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
