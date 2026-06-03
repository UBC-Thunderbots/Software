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
    : time_since_linear_trajectory_creation_(Duration::fromSeconds(0)),
      time_since_angular_trajectory_creation_(Duration::fromSeconds(0)),
      time_since_trajectory_creation_(Duration::fromSeconds(0)),
      robot_constants_(robot_constants)
{
}

void PrimitiveExecutor::updatePrimitive(const TbotsProto::Primitive& primitive_msg)
{
    current_primitive_ = primitive_msg;

    if (current_primitive_.has_move())
    {
        const std::optional new_trajectory_path =
            createTrajectoryPathFromParams(current_primitive_.move().xy_traj_params(),
                                           position_, velocity_, robot_constants_);

        const bool is_linear_traj_new =
            (new_trajectory_path.has_value() != trajectory_path_.has_value()) ||
            (new_trajectory_path.has_value() &&
             !trajectory_path_->equals(*new_trajectory_path,
                                       LINEAR_DESTINATION_THRESHOLD_METERS));

        if (is_linear_traj_new)
        {
            trajectory_path_                       = new_trajectory_path;
            time_since_linear_trajectory_creation_ = Duration::fromSeconds(0);
            position_controller_.reset();
        }

        const BangBangTrajectory1DAngular new_angular_trajectory =
            createAngularTrajectoryFromParams(current_primitive_.move().w_traj_params(),
                                              orientation_, angular_velocity_,
                                              robot_constants_);

        const bool is_angular_traj_new =
            !angular_trajectory_.has_value() ||
            !angular_trajectory_->equals(new_angular_trajectory,
                                         ANGULAR_DESTINATION_THRESHOLD_DEGREES);

        if (is_angular_traj_new)
        {
            angular_trajectory_                     = new_angular_trajectory;
            time_since_angular_trajectory_creation_ = Duration::fromSeconds(0);
            orientation_controller_.reset();
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

    if (!current_primitive_.has_move())
    {
        return;
    }

    // If we are lagging behind trajectory too much, we have stalled! We need to
    // regenerate trajectory.
    if (trajectory_path_.has_value())
    {
        const double linear_following_error =
            (position_ - trajectory_path_->getPosition(
                             time_since_linear_trajectory_creation_.toSeconds()))
                .length();

        if (linear_following_error > LINEAR_STALL_ERROR_MAX_METERS)
        {
            // regenerate linear trajectory
            trajectory_path_ =
                createTrajectoryPathFromParams(current_primitive_.move().xy_traj_params(),
                                               position_, velocity_, robot_constants_);

            time_since_linear_trajectory_creation_ = Duration::fromSeconds(0);
        }
    }

    if (angular_trajectory_.has_value())
    {
        const double angular_following_error =
            angular_trajectory_
                ->getPosition(time_since_angular_trajectory_creation_.toSeconds())
                .minDiff(orientation_)
                .toDegrees();

        if (angular_following_error > ANGULAR_STALL_ERROR_MAX_DEGREES)
        {
            // regenerate angular trajectory
            angular_trajectory_ = createAngularTrajectoryFromParams(
                current_primitive_.move().w_traj_params(), orientation_,
                angular_velocity_, robot_constants_);

            time_since_angular_trajectory_creation_ = Duration::fromSeconds(0);
        }
    }
}

Vector PrimitiveExecutor::getTargetLinearVelocity()
{
    Vector local_velocity = globalToLocalVelocity(
        trajectory_path_->getVelocity(time_since_trajectory_creation_.toSeconds()),
        orientation_);
    Point position =
        trajectory_path_->getPosition(time_since_trajectory_creation_.toSeconds());
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
    TbotsProto::PrimitiveExecutorStatus& status, const Duration& delta_time)
{
    time_since_trajectory_creation_ += delta_time;
    time_since_linear_trajectory_creation_ += delta_time;
    time_since_angular_trajectory_creation_ += delta_time;

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
                LOG(INFO) << "Not moving because trajectory_path_ has value "
                          << trajectory_path_.has_value() << " angular has "
                          << angular_trajectory_.has_value();
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
