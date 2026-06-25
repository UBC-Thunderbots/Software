#include "software/embedded/primitive_executor.h"

#include <algorithm>
#include <cmath>

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
    : state_(), current_primitive_(), robot_constants_(robot_constants)
{
}

void PrimitiveExecutor::updatePrimitive(const TbotsProto::Primitive& primitive_msg)
{
    current_primitive_ = primitive_msg;

    if (current_primitive_.has_move())
    {
        const auto new_trajectory_path =
            createTrajectoryPathFromParams(current_primitive_.move().xy_traj_params(),
                                           state_.velocity(), robot_constants_);

        const auto new_angular_trajectory =
            createAngularTrajectoryFromParams(current_primitive_.move().w_traj_params(),
                                              state_.angularVelocity(), robot_constants_);

        trajectory_path_ = new_trajectory_path;
        position_controller_.reset();
        time_since_linear_trajectory_creation_ =
            Duration::fromSeconds(VISION_TO_ROBOT_DELAY_S);

        angular_trajectory_ = new_angular_trajectory;
        orientation_controller_.reset();
        time_since_angular_trajectory_creation_ =
            Duration::fromSeconds(VISION_TO_ROBOT_DELAY_S);

        // Reset the forward-only reverse-driving state so each new move re-decides
        // whether to drive forwards or backwards from scratch.
        forward_only_reversing_ = false;
    }
}

void PrimitiveExecutor::updateState(const RobotState& state)
{
    state_ = state;
}

Vector PrimitiveExecutor::stepTargetLinearVelocity(const Duration& delta_time)
{
    Vector target_v_global =
        position_controller_.step(state_.position(), *trajectory_path_,
                                  time_since_linear_trajectory_creation_, delta_time);

    // make sure robot doesn't go faster than max speed (speed is frame-invariant)
    target_v_global = target_v_global.normalize(
        std::min(target_v_global.length(),
                 static_cast<double>(robot_constants_.robot_max_speed_m_per_s)));

    const Vector velocity_delta = target_v_global - prev_target_global_velocity_;
    const double max_velocity_delta =
        robot_constants_.robot_max_acceleration_m_per_s_2 * delta_time.toSeconds();
    if (velocity_delta.length() > max_velocity_delta)
    {
        target_v_global =
            prev_target_global_velocity_ + velocity_delta.normalize(max_velocity_delta);
    }
    prev_target_global_velocity_ = target_v_global;

    Vector target_v_local = globalToLocalVelocity(target_v_global, state_.orientation());

    if (ENABLE_FORWARD_ONLY_MOTION)
    {
        // Drop the sideways (strafe) component so the robot only drives along its
        // heading. The remaining local-x component is exactly the projection of the
        // desired global velocity onto the robot's facing direction: the robot moves at
        // (close to) the desired speed once it has rotated to face its direction of
        // travel, and barely translates while it is still turning to face it. The angular
        // controller (stepForwardOnlyTargetAngularVelocity) is what turns the robot to
        // face its travel direction.
        target_v_local.setY(0.0);

        // Keep the tracked "previous commanded velocity" consistent with what is actually
        // commanded, so the acceleration limiting above stays accurate on the next step.
        prev_target_global_velocity_ =
            localToGlobalVelocity(target_v_local, state_.orientation());
    }

    return target_v_local;
}

AngularVelocity PrimitiveExecutor::stepTargetAngularVelocity(const Duration& delta_time)
{
    AngularVelocity target_w;
    if (ENABLE_FORWARD_ONLY_MOTION)
    {
        // Slave the heading to the direction of travel (then to the final orientation
        // near the destination) instead of following the independently-planned angular
        // trajectory, so the robot only needs to move forwards/backwards.
        target_w = stepForwardOnlyTargetAngularVelocity();
    }
    else
    {
        target_w = orientation_controller_.step(
            state_.orientation(), *angular_trajectory_,
            time_since_angular_trajectory_creation_, delta_time);
    }

    // make sure robot doesn't rotate faster than max angular speed
    const double max_speed = robot_constants_.robot_max_ang_speed_rad_per_s;
    const double clamped_w = std::clamp(target_w.toRadians(), -max_speed, max_speed);
    target_w               = AngularVelocity::fromRadians(clamped_w);

    const double max_angular_velocity_delta =
        robot_constants_.robot_max_ang_acceleration_rad_per_s_2 * delta_time.toSeconds();
    const double angular_velocity_delta =
        std::clamp((target_w - prev_target_angular_velocity_).toRadians(),
                   -max_angular_velocity_delta, max_angular_velocity_delta);
    target_w                      = prev_target_angular_velocity_ +
                                    AngularVelocity::fromRadians(angular_velocity_delta);
    prev_target_angular_velocity_ = target_w;
    return target_w;
}

AngularVelocity PrimitiveExecutor::stepForwardOnlyTargetAngularVelocity()
{
    const Angle orientation = state_.orientation();
    const double elapsed_s  = time_since_linear_trajectory_creation_.toSeconds();

    Angle target_orientation;
    const double distance_to_destination =
        distance(state_.position(), trajectory_path_->getDestination());

    if (distance_to_destination > FORWARD_ONLY_FINAL_ROTATION_DISTANCE_M)
    {
        // Pure-pursuit style heading: aim at a look-ahead point further along the planned
        // path, and steer towards it. The vector from the robot's actual position to that
        // point both follows the path's curvature and steers back onto the path when the
        // robot has drifted off it (cross-track error) -- which the robot cannot fix by
        // strafing. Anchoring to a point on the path (rather than the instantaneous
        // desired velocity) and looking ahead provides damping, so the robot converges
        // onto the path smoothly instead of weaving across it.
        const Point lookahead_point =
            trajectory_path_->getPosition(elapsed_s + FORWARD_ONLY_LOOKAHEAD_TIME_S);
        Vector travel_direction = lookahead_point - state_.position();
        if (travel_direction.length() < 1e-3)
        {
            // The look-ahead point coincides with the robot (e.g. at the very start from
            // rest). Fall back to the straight-line direction towards the destination.
            travel_direction = trajectory_path_->getDestination() - state_.position();
        }

        const Angle forward_heading = travel_direction.orientation();
        const Angle reverse_heading = forward_heading + Angle::half();

        // Pick driving forwards or backwards, whichever needs a smaller turn, with
        // hysteresis so the decision doesn't chatter when the travel direction is roughly
        // perpendicular to the robot.
        const Angle forward_turn = orientation.minDiff(forward_heading);
        const Angle reverse_turn = orientation.minDiff(reverse_heading);
        const Angle hysteresis = Angle::fromRadians(FORWARD_ONLY_REVERSE_HYSTERESIS_RAD);
        if (forward_only_reversing_)
        {
            forward_only_reversing_ = !(forward_turn + hysteresis < reverse_turn);
        }
        else
        {
            forward_only_reversing_ = (reverse_turn + hysteresis < forward_turn);
        }
        target_orientation = forward_only_reversing_ ? reverse_heading : forward_heading;
    }
    else
    {
        // Close to the destination: rotate to the requested final orientation. The
        // angular trajectory's destination is the final_angle requested by the primitive.
        target_orientation = angular_trajectory_->getDestination();
    }

    const Angle error = (target_orientation - orientation).clamp();

    // Deadband so the robot fully settles (and doesn't jitter on sensor noise) once it is
    // close enough to the target orientation.
    if (error.abs().toRadians() < FORWARD_ONLY_HEADING_DEADBAND_RAD)
    {
        return AngularVelocity::zero();
    }

    // Saturated proportional controller toward the target orientation. The returned value
    // is clamped to the max angular speed (and acceleration) by
    // stepTargetAngularVelocity, so this saturates to a max-speed turn when far away and
    // decays proportionally as the error shrinks. This gives a first-order, well-damped
    // response that settles on the target. We deliberately avoid a time-optimal
    // deceleration profile (|w| = sqrt(2 * decel * error)): that sits on the edge of
    // stability and tends to overshoot and oscillate around the target once real-world
    // latency and discretization are involved, since it keeps commanding a large angular
    // speed even very close to the target.
    return AngularVelocity::fromRadians(FORWARD_ONLY_HEADING_KP * error.toRadians());
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    TbotsProto::PrimitiveExecutorStatus& status, const Duration& delta_time)
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
    prev_target_global_velocity_ =
        localToGlobalVelocity(local_velocity, state_.orientation());
    prev_target_angular_velocity_ = angular_velocity;
}

void PrimitiveExecutor::sendLinearMotionToPlotJuggler(const Vector& target_local_velocity,
                                                      const Duration& delta_time) const
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
