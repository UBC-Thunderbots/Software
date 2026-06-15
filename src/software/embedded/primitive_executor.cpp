#include "software/embedded/primitive_executor.h"

#include <algorithm>

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
    : current_primitive_(), robot_constants_(robot_constants)
{
}

bool PrimitiveExecutor::isLinearTrajectoryNew(
    const std::optional<TrajectoryPath>& new_trajectory) const
{
    if (new_trajectory.has_value() != trajectory_path_.has_value())
    {
        // We either don't have a trajectory and the new one does, or we have one and
        // the new one doesn't.
        return true;
    }

    if (!new_trajectory.has_value())
    {
        return false;
    }

    // Regenerate if the destination moved meaningfully. This also catches changes
    // beyond the comparison horizon below (e.g. a far-away destination shifting).
    if (!trajectory_path_->equals(*new_trajectory, LINEAR_DESTINATION_THRESHOLD_METERS))
    {
        return true;
    }

    // The destinations match, but the path to get there may have changed (e.g. a
    // newly-detected obstacle forces a detour). Compare the two plans over the same
    // upcoming time window, anchored at where we currently are on the old trajectory,
    // so timing/speed differences are accounted for (a purely spatial comparison would
    // not). Regenerate if the plans ever diverge by more than the deviation threshold.
    const double old_time = time_since_linear_trajectory_creation_.toSeconds();
    const double new_time = VISION_TO_ROBOT_DELAY_S;
    const double horizon  = std::min(std::max(trajectory_path_->getTotalTime() - old_time,
                                              new_trajectory->getTotalTime() - new_time),
                                     TRAJECTORY_DEVIATION_HORIZON_S);

    for (double tau = 0.0; tau <= horizon; tau += TRAJECTORY_DEVIATION_TIME_STEP_S)
    {
        const Point old_pos = trajectory_path_->getPosition(old_time + tau);
        const Point new_pos = new_trajectory->getPosition(new_time + tau);
        if (distance(old_pos, new_pos) > LINEAR_TRAJECTORY_DEVIATION_THRESHOLD_METERS)
        {
            return true;
        }
    }

    return false;
}

bool PrimitiveExecutor::isAngularTrajectoryNew(
    const BangBangTrajectory1DAngular& new_trajectory) const
{
    if (!angular_trajectory_.has_value())
    {
        return true;
    }

    // Regenerate if the destination rotated meaningfully (also catches changes beyond
    // the comparison horizon below).
    if (!angular_trajectory_->equals(new_trajectory,
                                     ANGULAR_DESTINATION_THRESHOLD_DEGREES))
    {
        return true;
    }

    // Compare the two plans over the same upcoming time window (see isLinearTrajectoryNew
    // for rationale), regenerating if they ever diverge by more than the threshold.
    const double old_time = time_since_angular_trajectory_creation_.toSeconds();
    const double new_time = VISION_TO_ROBOT_DELAY_S;
    const double horizon =
        std::min(std::max(angular_trajectory_->getTotalTime() - old_time,
                          new_trajectory.getTotalTime() - new_time),
                 TRAJECTORY_DEVIATION_HORIZON_S);

    for (double tau = 0.0; tau <= horizon; tau += TRAJECTORY_DEVIATION_TIME_STEP_S)
    {
        const Angle old_orientation = angular_trajectory_->getPosition(old_time + tau);
        const Angle new_orientation = new_trajectory.getPosition(new_time + tau);
        if (old_orientation.minDiff(new_orientation).toDegrees() >
            ANGULAR_TRAJECTORY_DEVIATION_THRESHOLD_DEGREES)
        {
            return true;
        }
    }

    return false;
}

void PrimitiveExecutor::updatePrimitive(const TbotsProto::Primitive& primitive_msg)
{
    current_primitive_ = primitive_msg;

    if (current_primitive_.has_move())
    {
        const auto new_trajectory_path =
            createTrajectoryPathFromParams(current_primitive_.move().xy_traj_params(),
                                           state_.velocity(), robot_constants_);

        // Only regenerate the trajectory (and reset the controller/clock) when the
        // destination has meaningfully changed. The AI re-sends the same primitive
        // every tick; regenerating unconditionally would pin the robot to the start
        // of a fresh trajectory forever, so it would never accelerate along it.
        if (isLinearTrajectoryNew(new_trajectory_path))
        {
            trajectory_path_ = new_trajectory_path;
            position_controller_.reset();
            time_since_linear_trajectory_creation_ =
                Duration::fromSeconds(VISION_TO_ROBOT_DELAY_S);
        }

        const auto new_angular_trajectory =
            createAngularTrajectoryFromParams(current_primitive_.move().w_traj_params(),
                                              state_.angularVelocity(), robot_constants_);

        if (isAngularTrajectoryNew(new_angular_trajectory))
        {
            angular_trajectory_ = new_angular_trajectory;
            orientation_controller_.reset();
            time_since_angular_trajectory_creation_ =
                Duration::fromSeconds(VISION_TO_ROBOT_DELAY_S);
        }
    }
}

void PrimitiveExecutor::updateState(const RobotState& state)
{
    state_ = state;
}

Vector PrimitiveExecutor::stepTargetLinearVelocity(Duration delta_time)
{
    const auto target_v_global =
        position_controller_.step(state_.position(), *trajectory_path_,
                                  time_since_linear_trajectory_creation_, delta_time);
    auto target_v_local = globalToLocalVelocity(target_v_global, state_.orientation());

    // make sure robot doesn't go faster than max speed
    target_v_local = target_v_local.normalize(
        std::min(target_v_local.length(),
                 static_cast<double>(robot_constants_.robot_max_speed_m_per_s)));

    const Vector local_acceleration =
        (target_v_local - prev_target_local_velocity_) / delta_time.toSeconds();

    if (local_acceleration.length() > robot_constants_.robot_max_acceleration_m_per_s_2)
    {
        LOG(WARNING) << "Robot trying to accelerate at " << local_acceleration.length()
                     << "m/s^2.";
    }

    prev_target_local_velocity_ = target_v_local;
    return target_v_local;
}

AngularVelocity PrimitiveExecutor::stepTargetAngularVelocity(Duration delta_time)
{
    auto target_w =
        orientation_controller_.step(state_.orientation(), *angular_trajectory_,
                                     time_since_angular_trajectory_creation_, delta_time);

    // make sure robot doesn't rotate faster than max angular speed
    const double max_speed = robot_constants_.robot_max_ang_speed_rad_per_s;
    const double clamped_w = std::clamp(target_w.toRadians(), -max_speed, max_speed);
    target_w               = AngularVelocity::fromRadians(clamped_w);

    const auto angular_acceleration =
        (target_w - prev_target_angular_velocity_) / delta_time.toSeconds();
    if (std::abs(angular_acceleration.toRadians()) >
        robot_constants_.robot_max_ang_acceleration_rad_per_s_2)
    {
        LOG(WARNING) << "Robot trying to angular accelerate at "
                     << angular_acceleration.toRadians() << "rads/s^2.";
    }
    prev_target_angular_velocity_ = target_w;
    return target_w;
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    TbotsProto::PrimitiveExecutorStatus& status, Duration delta_time)
{
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
            setPrevCommandedVelocity(Vector(), AngularVelocity());
            return output;
        }
        case TbotsProto::Primitive::kDirectControl:
        {
            const auto& motor_control =
                current_primitive_.direct_control().motor_control();
            if (motor_control.has_direct_velocity_control())
            {
                setPrevCommandedVelocity(
                    createVector(motor_control.direct_velocity_control().velocity()),
                    createAngularVelocity(
                        motor_control.direct_velocity_control().angular_velocity()));
            }
            else
            {
                setPrevCommandedVelocity(Vector(), AngularVelocity());
            }
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
                setPrevCommandedVelocity(Vector(), AngularVelocity());
                return output;
            }

            Vector local_velocity            = stepTargetLinearVelocity(delta_time);
            AngularVelocity angular_velocity = stepTargetAngularVelocity(delta_time);

            // For debugging:
            // sendLinearMotionToPlotJuggler(local_velocity, delta_time);

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
    setPrevCommandedVelocity(Vector(), AngularVelocity());
    return std::make_unique<TbotsProto::DirectControlPrimitive>();
}

void PrimitiveExecutor::setPrevCommandedVelocity(const Vector& local_velocity,
                                                 const AngularVelocity& angular_velocity)
{
    prev_target_local_velocity_   = local_velocity;
    prev_target_angular_velocity_ = angular_velocity;
}

void PrimitiveExecutor::sendLinearMotionToPlotJuggler(const Vector& target_local_velocity,
                                                      Duration delta_time)
{
    const Vector& local_acceleration =
        (target_local_velocity - state_.localVelocity()) / delta_time.toSeconds();
    LOG(PLOTJUGGLER) << *createPlotJugglerValue({{"x", state_.position().x()},
                                                 {"y", state_.position().y()},
                                                 {"v_x", target_local_velocity.x()},
                                                 {"v_y", target_local_velocity.y()},
                                                 {"a_x", local_acceleration.x()},
                                                 {"a_y", local_acceleration.y()}});
}
