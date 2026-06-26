#include "software/embedded/primitive_executor.h"

#include <algorithm>
#include <vector>

#include "proto/message_translation/tbots_geometry.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"
#include "software/physics/velocity_conversion_util.h"

namespace
{
/**
 * Check whether the destinations and subdestinations of two trajectory paths have
 * changed significantly enough to warrant switching to the new trajectory.
 *
 * Compares the final destination (end of the last trajectory node) and the end position
 * of each intermediate trajectory path node (subdestination). If any corresponding
 * destination differs by more than threshold_m, or the number of subdestinations differs,
 * the paths are considered to have diverged.
 *
 * @param current The trajectory path currently being followed
 * @param new_trajectory The newly received trajectory path
 * @param threshold_m Maximum allowed difference in metres
 * @return true if destinations or subdestinations differ by more than threshold_m
 */
bool destinationsChangedSignificantly(const TrajectoryPath& current,
                                      const TrajectoryPath& new_trajectory,
                                      double threshold_m)
{
    // Compare final destinations
    const Point current_dest = current.getPosition(current.getTotalTime());
    const Point new_dest     = new_trajectory.getPosition(new_trajectory.getTotalTime());

    if ((current_dest - new_dest).length() > threshold_m)
    {
        return true;
    }

    // Compare subdestinations (end positions of each trajectory path node)
    const auto& current_nodes = current.getTrajectoryPathNodes();
    const auto& new_nodes     = new_trajectory.getTrajectoryPathNodes();

    if (current_nodes.size() != new_nodes.size())
    {
        return true;
    }

    for (size_t i = 0; i < current_nodes.size(); ++i)
    {
        const Point current_subdest = current_nodes[i].getTrajectory()->getPosition(
            current_nodes[i].getTrajectoryEndTime());
        const Point new_subdest = new_nodes[i].getTrajectory()->getPosition(
            new_nodes[i].getTrajectoryEndTime());

        if ((current_subdest - new_subdest).length() > threshold_m)
        {
            return true;
        }
    }

    return false;
}
}  // namespace

PrimitiveExecutor::PrimitiveExecutor(
    const robot_constants::RobotConstants& robot_constants)
    : state_(), current_primitive_(), robot_constants_(robot_constants)
{
}

void PrimitiveExecutor::updatePrimitive(const TbotsProto::Primitive& primitive_msg)
{
    current_primitive_ = primitive_msg;

    if (!current_primitive_.has_move())
    {
        return;
    }

    const std::optional<TrajectoryPath> new_trajectory_path =
        createTrajectoryPathFromParams(current_primitive_.move().xy_traj_params(),
                                       state_.velocity(), robot_constants_);

    const BangBangTrajectory1DAngular new_angular_trajectory =
        createAngularTrajectoryFromParams(current_primitive_.move().w_traj_params(),
                                          state_.angularVelocity(), robot_constants_);

    if (!new_trajectory_path.has_value())
    {
        // No feasible linear trajectory (e.g. the max allowed speed is zero); stop
        // following any linear trajectory so the robot holds still.
        trajectory_path_.reset();
        prev_trajectory_path_.reset();
        linear_blend_remaining_ = Duration::fromSeconds(0);
    }
    else if (shouldFollowNewLinearTrajectory(*new_trajectory_path))
    {
        startFollowingNewLinearTrajectory(*new_trajectory_path);
    }

    if (shouldFollowNewAngularTrajectory(new_angular_trajectory))
    {
        startFollowingNewAngularTrajectory(new_angular_trajectory);
    }
}

bool PrimitiveExecutor::shouldFollowNewLinearTrajectory(
    const TrajectoryPath& new_trajectory) const
{
    // If we aren't following any trajectory yet, we must start following the new one
    if (!trajectory_path_.has_value())
    {
        return true;
    }

    // Compare subdestinations and destinations of current and new trajectory.
    // If they've changed significantly, switch to the new trajectory.
    if (destinationsChangedSignificantly(*trajectory_path_, new_trajectory,
                                         DESTINATION_THRESHOLD_M))
    {
        return true;
    }

    // Measure how well we're tracking the current trajectory by comparing the robot's
    // actual state against where the trajectory expects it to be at the current elapsed
    // time. If we are already tracking it well (position and velocity errors are small),
    // keep following it to avoid unnecessary controller resets.
    const double elapsed_time_sec = trajectory_elapsed_time_.toSeconds();
    const double position_error =
        (state_.position() - trajectory_path_->getPosition(elapsed_time_sec)).length();
    const double velocity_error =
        (state_.velocity() - trajectory_path_->getVelocity(elapsed_time_sec)).length();

    if (position_error < POSITION_TRACKING_THRESHOLD_M &&
        velocity_error < VELOCITY_TRACKING_THRESHOLD_M_PER_S)
    {
        return false;
    }

    return true;
}

bool PrimitiveExecutor::shouldFollowNewAngularTrajectory(
    const BangBangTrajectory1DAngular& new_trajectory) const
{
    // If we aren't following any trajectory yet, we must start following the new one
    if (!angular_trajectory_.has_value())
    {
        return true;
    }

    // Switch to new trajectory if destination has meaningfully changed
    const Angle current_final_orientation =
        angular_trajectory_->getPosition(angular_trajectory_->getTotalTime());
    const Angle new_final_orientation =
        new_trajectory.getPosition(new_trajectory.getTotalTime());
    if (current_final_orientation.minDiff(new_final_orientation).toDegrees() >
        ANGULAR_DESTINATION_THRESHOLD_DEGREES)
    {
        return true;
    }

    return false;
}

void PrimitiveExecutor::startFollowingNewLinearTrajectory(
    const TrajectoryPath& new_trajectory)
{
    // If we were already following a trajectory, retain it for a short blend window so
    // we can crossfade its velocity setpoint into the new trajectory's, avoiding an
    // abrupt change in commanded velocity.
    if (trajectory_path_.has_value())
    {
        prev_trajectory_path_         = trajectory_path_;
        prev_trajectory_elapsed_time_ = trajectory_elapsed_time_;
        linear_blend_remaining_ = Duration::fromSeconds(TRAJECTORY_BLEND_DURATION_S);
    }

    trajectory_path_         = new_trajectory;
    trajectory_elapsed_time_ = Duration::fromSeconds(0);
    position_controller_.reset();
    velocity_x_pid_.reset();
    velocity_y_pid_.reset();
}

void PrimitiveExecutor::startFollowingNewAngularTrajectory(
    const BangBangTrajectory1DAngular& new_trajectory)
{
    // See startFollowingNewLinearTrajectory for the blend rationale.
    if (angular_trajectory_.has_value())
    {
        prev_angular_trajectory_              = angular_trajectory_;
        prev_angular_trajectory_elapsed_time_ = angular_trajectory_elapsed_time_;
        angular_blend_remaining_ = Duration::fromSeconds(TRAJECTORY_BLEND_DURATION_S);
    }

    angular_trajectory_              = new_trajectory;
    angular_trajectory_elapsed_time_ = Duration::fromSeconds(0);
    orientation_controller_.reset();
}

void PrimitiveExecutor::updateState(const RobotState& state)
{
    state_ = state;
}

Vector PrimitiveExecutor::stepTargetLinearVelocity(const Duration& delta_time)
{
    // Progress the trajectory purely by time: advance the elapsed time by this step's
    // delta. Sample a small look-ahead past the elapsed time so the target leads the
    // robot slightly. Both the elapsed time and the sample are clamped to the end of the
    // trajectory.
    trajectory_elapsed_time_ =
        std::min(trajectory_elapsed_time_ + delta_time,
                 Duration::fromSeconds(trajectory_path_->getTotalTime()));
    const double sample_time_sec =
        std::min(trajectory_elapsed_time_.toSeconds() + TRAJECTORY_LOOKAHEAD_TIME_S,
                 trajectory_path_->getTotalTime());

    const Point target_position  = trajectory_path_->getPosition(sample_time_sec);
    const Vector target_velocity = trajectory_path_->getVelocity(sample_time_sec);

    Vector target_v_global =
        position_controller_.step(state_.position(), *trajectory_path_,
                                  Duration::fromSeconds(sample_time_sec), delta_time);

    // Add a velocity-tracking PID correction on top of the position-based target.
    // This helps the robot match the trajectory's feedforward velocity, reducing
    // lag that the position PID alone may not correct quickly enough.
    const Vector velocity_error =
        trajectory_path_->getVelocity(sample_time_sec) - state_.velocity();
    target_v_global +=
        Vector(velocity_x_pid_.step(velocity_error.x(), delta_time.toSeconds()),
               velocity_y_pid_.step(velocity_error.y(), delta_time.toSeconds()));

    // Smoothly blend the velocity setpoint from the trajectory we just switched away
    // from into the new one over a short window, so it doesn't jump on the switch. The
    // previous trajectory's velocity is sampled at its own elapsed time, which keeps
    // advancing during the blend.
    if (linear_blend_remaining_.toSeconds() > 0.0 && prev_trajectory_path_.has_value())
    {
        prev_trajectory_elapsed_time_ =
            std::min(prev_trajectory_elapsed_time_ + delta_time,
                     Duration::fromSeconds(prev_trajectory_path_->getTotalTime()));
        const double prev_sample_time_sec = std::min(
            prev_trajectory_elapsed_time_.toSeconds() + TRAJECTORY_LOOKAHEAD_TIME_S,
            prev_trajectory_path_->getTotalTime());
        const Vector prev_traj_velocity =
            prev_trajectory_path_->getVelocity(prev_sample_time_sec);

        // alpha ramps from 0 (just switched: follow the old trajectory) to 1 (blend
        // finished: fully follow the new trajectory).
        const double alpha = std::clamp(
            1.0 - linear_blend_remaining_.toSeconds() / TRAJECTORY_BLEND_DURATION_S, 0.0,
            1.0);
        target_v_global = prev_traj_velocity * (1.0 - alpha) + target_v_global * alpha;

        linear_blend_remaining_ -= delta_time;
        if (linear_blend_remaining_.toSeconds() <= 0.0)
        {
            prev_trajectory_path_.reset();
        }
    }

    // make sure robot doesn't go faster than max speed (speed is frame-invariant)
    target_v_global = target_v_global.normalize(
        std::min(target_v_global.length(),
                 static_cast<double>(robot_constants_.robot_max_speed_m_per_s)));

    // The trajectory's own velocity is acceleration-bounded, but the PID correction and
    // per-tick trajectory regeneration are added on top, so the emitted command must be
    // re-limited here to the robot's kinematic acceleration limit. Otherwise, the
    // commanded velocity can step far more than robot_max_acceleration * delta_time per
    // tick, asking the robot to accelerate well beyond what it (and the motors/SPI) can
    // sustain.
    //
    // The limit applies to the robot's translational (global-frame) velocity. Measuring
    // the change in the rotating body frame (i.e. after globalToLocalVelocity) would add
    // the v*omega term from the body frame spinning, which falsely trips the limit
    // whenever the robot translates while rotating even though its global motion is
    // within limits. So clamp the change in global velocity, relative to the previous
    // commanded (not measured) velocity.
    //
    // We use the robot's physical acceleration/deceleration limits here (not the
    // possibly-slower limits used to generate the trajectory): the trajectory is planned
    // conservatively to leave headroom, but the PID is allowed to use the full physical
    // capability to correct lag. Whether we're speeding up or slowing down selects the
    // acceleration or deceleration limit respectively.
    const Vector velocity_delta = target_v_global - prev_target_global_velocity_;
    const bool is_decelerating =
        target_v_global.length() < prev_target_global_velocity_.length();
    const double max_accel_m_per_s_2 =
        is_decelerating ? robot_constants_.robot_max_deceleration_m_per_s_2
                        : robot_constants_.robot_max_acceleration_m_per_s_2;
    const double max_velocity_delta = max_accel_m_per_s_2 * delta_time.toSeconds();
    if (velocity_delta.length() > max_velocity_delta)
    {
        target_v_global =
            prev_target_global_velocity_ + velocity_delta.normalize(max_velocity_delta);
    }

    prev_target_global_velocity_ = target_v_global;

    LOG(PLOTJUGGLER) << *createPlotJugglerValue({{"target_pos_x", target_position.x()},
                                                 {"target_pos_y", target_position.y()},
                                                 {"target_vel_x", target_v_global.x()},
                                                 {"target_vel_y", target_v_global.y()},
                                                 {"actual_pos_x", state_.position().x()},
                                                 {"actual_pos_y", state_.position().y()},
                                                 {"actual_vel_x", state_.velocity().x()},
                                                 {"actual_vel_y", state_.velocity().y()},
                                                 {"traj_vel_x", target_velocity.x()},
                                                 {"traj_vel_y", target_velocity.y()}});

    return globalToLocalVelocity(target_v_global, state_.orientation());
}

AngularVelocity PrimitiveExecutor::stepTargetAngularVelocity(const Duration& delta_time)
{
    // Progress the angular trajectory purely by time, mirroring stepTargetLinearVelocity,
    // including the small look-ahead on the sample.
    angular_trajectory_elapsed_time_ =
        std::min(angular_trajectory_elapsed_time_ + delta_time,
                 Duration::fromSeconds(angular_trajectory_->getTotalTime()));
    const double sample_time_sec = std::min(
        angular_trajectory_elapsed_time_.toSeconds() + TRAJECTORY_LOOKAHEAD_TIME_S,
        angular_trajectory_->getTotalTime());

    LOG(PLOTJUGGLER) << *createPlotJugglerValue({
        {"target_orientation_rad",
         angular_trajectory_->getPosition(sample_time_sec).toRadians()},
        {"target_angular_vel_rad_per_s",
         angular_trajectory_->getVelocity(sample_time_sec).toRadians()},
        {"actual_orientation_rad", state_.orientation().toRadians()},
        {"actual_angular_vel_rad_per_s", state_.angularVelocity().toRadians()},
    });

    auto target_w =
        orientation_controller_.step(state_.orientation(), *angular_trajectory_,
                                     Duration::fromSeconds(sample_time_sec), delta_time);

    // Smoothly blend the angular velocity setpoint from the trajectory we just switched
    // away from into the new one over a short window, so it doesn't jump on the switch.
    if (angular_blend_remaining_.toSeconds() > 0.0 &&
        prev_angular_trajectory_.has_value())
    {
        prev_angular_trajectory_elapsed_time_ =
            std::min(prev_angular_trajectory_elapsed_time_ + delta_time,
                     Duration::fromSeconds(prev_angular_trajectory_->getTotalTime()));
        const double prev_sample_time_sec =
            std::min(prev_angular_trajectory_elapsed_time_.toSeconds() +
                         TRAJECTORY_LOOKAHEAD_TIME_S,
                     prev_angular_trajectory_->getTotalTime());
        const AngularVelocity prev_traj_w =
            prev_angular_trajectory_->getVelocity(prev_sample_time_sec);

        // alpha ramps from 0 (just switched: follow the old trajectory) to 1 (blend
        // finished: fully follow the new trajectory).
        const double alpha = std::clamp(
            1.0 - angular_blend_remaining_.toSeconds() / TRAJECTORY_BLEND_DURATION_S, 0.0,
            1.0);
        target_w = prev_traj_w * (1.0 - alpha) + target_w * alpha;

        angular_blend_remaining_ -= delta_time;
        if (angular_blend_remaining_.toSeconds() <= 0.0)
        {
            prev_angular_trajectory_.reset();
        }
    }

    // make sure robot doesn't rotate faster than max angular speed
    const double max_speed = robot_constants_.robot_max_ang_speed_rad_per_s;
    const double clamped_w = std::clamp(target_w.toRadians(), -max_speed, max_speed);
    target_w               = AngularVelocity::fromRadians(clamped_w);

    // Re-limit the commanded angular velocity to the robot's angular acceleration limit,
    // for the same reason as the translational clamp above: the feedforward trajectory is
    // acceleration-bounded, but the PID correction and per-tick regeneration are added on
    // top, so the emitted command can step more than robot_max_ang_acceleration *
    // delta_time per tick.
    const double max_angular_velocity_delta =
        robot_constants_.robot_max_ang_acceleration_rad_per_s_2 * delta_time.toSeconds();
    const double angular_velocity_delta =
        std::clamp((target_w - prev_target_angular_velocity_).toRadians(),
                   -max_angular_velocity_delta, max_angular_velocity_delta);
    target_w = prev_target_angular_velocity_ +
               AngularVelocity::fromRadians(angular_velocity_delta);
    prev_target_angular_velocity_ = target_w;
    return target_w;
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    TbotsProto::PrimitiveExecutorStatus& status, const Duration& delta_time)
{
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
    prev_target_global_velocity_ =
        localToGlobalVelocity(local_velocity, state_.orientation());
    prev_target_angular_velocity_ = angular_velocity;
}
