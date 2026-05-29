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
    : last_step_time_(std::chrono::steady_clock::now()),
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
             trajectory_path_->isNew(*new_trajectory_path,
                                     LINEAR_DESTINATION_THRESHOLD_METERS));

        if (is_linear_traj_new)
        {
            trajectory_path_ = new_trajectory_path;
            time_since_linear_trajectory_creation_ =
                std::chrono::duration<double>::zero();
            position_controller_.reset();
        }

        const BangBangTrajectory1DAngular new_angular_trajectory =
            createAngularTrajectoryFromParams(current_primitive_.move().w_traj_params(),
                                              orientation_, angular_velocity_,
                                              robot_constants_);

        const bool is_angular_traj_new =
            !angular_trajectory_.has_value() ||
            angular_trajectory_->isNew(new_angular_trajectory,
                                        ANGULAR_DESTINATION_THRESHOLD_DEGREES);

        if (is_angular_traj_new)
        {
            angular_trajectory_ = new_angular_trajectory;
            time_since_angular_trajectory_creation_ =
                std::chrono::duration<double>::zero();
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

    // If we are lagging behind trajectory too much, we have stalled! We need to
    // regenerate trajectory.
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

std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    TbotsProto::PrimitiveExecutorStatus& status)
{
    const auto current_time = std::chrono::steady_clock::now();
    const auto delta_time_chrono = current_time - last_step_time_;
    const Duration delta_time = Duration::fromSeconds(
        std::chrono::duration<double>(delta_time_chrono).count());

    time_since_linear_trajectory_creation_ += delta_time_chrono;
    time_since_angular_trajectory_creation_ += delta_time_chrono;
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

            const Duration elapsed_linear = Duration::fromSeconds(
                time_since_linear_trajectory_creation_.count());
            const Duration elapsed_angular = Duration::fromSeconds(
                time_since_angular_trajectory_creation_.count());

            const Vector target_linear_velocity_global = position_controller_.step(
                position_, *trajectory_path_, elapsed_linear, delta_time);

            const AngularVelocity target_angular_velocity = orientation_controller_.step(
                orientation_, *angular_trajectory_, elapsed_angular, delta_time);

            Vector target_linear_velocity_local =
                globalToLocalVelocity(target_linear_velocity_global, orientation_);

            // Make sure target linear velocity is clamped
            target_linear_velocity_local = target_linear_velocity_local.normalize(
                std::min(target_linear_velocity_local.length(),
                         static_cast<double>(robot_constants_.robot_max_speed_m_per_s)));

            const auto prim = createDirectControlPrimitive(
                target_linear_velocity_local, target_angular_velocity,
                convertDribblerModeToDribblerSpeed(
                    current_primitive_.move().dribbler_mode(), robot_constants_),
                current_primitive_.move().auto_chip_or_kick());

            return std::make_unique<TbotsProto::DirectControlPrimitive>(
                prim->direct_control());
        }
        case TbotsProto::Primitive::PRIMITIVE_NOT_SET:
        {
        }
    }
    return std::make_unique<TbotsProto::DirectControlPrimitive>();
}
