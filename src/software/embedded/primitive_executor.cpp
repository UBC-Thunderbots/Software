#include "software/embedded/primitive_executor.h"

#include <Tracy.hpp>

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
    const robot_constants::RobotConstants& robot_constants, const RobotId robot_id)
    : robot_state_(), robot_constants_(robot_constants), robot_id_(robot_id)
{
}

void PrimitiveExecutor::updatePrimitive(const TbotsProto::Primitive& primitive_msg,
                                        TbotsProto::RobotStatus& robot_status)
{
    ZoneNamedN(_tracy_update_primitive, "Thunderloop: Update Primitive", true);

    const auto update_start = std::chrono::steady_clock::now();

    current_primitive_ = primitive_msg;

    if (current_primitive_.has_move())
    {
        const auto new_trajectory_path =
            createTrajectoryPathFromParams(current_primitive_.move().xy_traj_params(),
                                           robot_state_.velocity(), robot_constants_);

        const auto new_angular_trajectory = createAngularTrajectoryFromParams(
            current_primitive_.move().w_traj_params(), robot_state_.angularVelocity(),
            robot_constants_);

        trajectory_path_ = new_trajectory_path;
        position_controller_.reset();
        time_since_linear_trajectory_creation_s_ = VISION_TO_ROBOT_DELAY_S;

        angular_trajectory_ = new_angular_trajectory;
        orientation_controller_.reset();
        time_since_angular_trajectory_creation_s_ = VISION_TO_ROBOT_DELAY_S;
    }

    const auto update_end = std::chrono::steady_clock::now();
    using Millis          = std::chrono::duration<double, std::milli>;
    const Millis update_time =
        std::chrono::duration_cast<Millis>(update_end - update_start);

    robot_status.mutable_thunderloop_status()->set_primitive_executor_update_time_ms(
        update_time.count());
}

void PrimitiveExecutor::updateRobotState(const RobotState& robot_state)
{
    robot_state_ = robot_state;
}

Vector PrimitiveExecutor::stepTargetLinearVelocity(const double delta_time_s)
{
    Vector target_v_global =
        position_controller_.step(robot_state_.position(), *trajectory_path_,
                                  time_since_linear_trajectory_creation_s_, delta_time_s);

    // make sure robot doesn't go faster than max speed (speed is frame-invariant)
    target_v_global = target_v_global.normalize(
        std::min(target_v_global.length(),
                 static_cast<double>(robot_constants_.robot_max_speed_m_per_s)));

    const Vector velocity_delta = target_v_global - prev_target_global_velocity_;
    const double max_velocity_delta =
        robot_constants_.robot_max_acceleration_m_per_s_2 * delta_time_s;
    if (velocity_delta.length() > max_velocity_delta)
    {
        target_v_global =
            prev_target_global_velocity_ + velocity_delta.normalize(max_velocity_delta);
    }
    prev_target_global_velocity_ = target_v_global;

    return globalToLocalVelocity(target_v_global, robot_state_.orientation());
}

AngularVelocity PrimitiveExecutor::stepTargetAngularVelocity(const double delta_time_s)
{
    auto target_w = orientation_controller_.step(
        robot_state_.orientation(), *angular_trajectory_,
        time_since_angular_trajectory_creation_s_, delta_time_s);

    // make sure robot doesn't rotate faster than max angular speed
    const double max_speed = robot_constants_.robot_max_ang_speed_rad_per_s;
    const double clamped_w = std::clamp(target_w.toRadians(), -max_speed, max_speed);
    target_w               = AngularVelocity::fromRadians(clamped_w);

    const double max_angular_velocity_delta =
        robot_constants_.robot_max_ang_acceleration_rad_per_s_2 * delta_time_s;
    const double angular_velocity_delta =
        std::clamp((target_w - prev_target_angular_velocity_).toRadians(),
                   -max_angular_velocity_delta, max_angular_velocity_delta);
    target_w = prev_target_angular_velocity_ +
               AngularVelocity::fromRadians(angular_velocity_delta);
    prev_target_angular_velocity_ = target_w;
    return target_w;
}


TbotsProto::DirectControlPrimitive PrimitiveExecutor::stepPrimitive(
    TbotsProto::RobotStatus& robot_status, const double delta_time_s)
{
    ZoneNamedN(_tracy_step_primitive, "Thunderloop: Step Primitive", true);

    const auto step_start = std::chrono::steady_clock::now();

    TbotsProto::PrimitiveExecutorStatus& prim_exec_status =
        *(robot_status.mutable_primitive_executor_status());

    time_since_linear_trajectory_creation_s_ += delta_time_s;
    time_since_angular_trajectory_creation_s_ += delta_time_s;
    prim_exec_status.set_running_primitive(true);

    TbotsProto::DirectControlPrimitive output;

    switch (current_primitive_.primitive_case())
    {
        case TbotsProto::Primitive::kStop:
        {
            auto prim = createDirectControlPrimitive(Vector(), AngularVelocity(), 0.0,
                                                     TbotsProto::AutoChipOrKick());
            output    = prim->direct_control();
            prim_exec_status.set_running_primitive(false);
            setPrevCommandedVelocity(Vector(), AngularVelocity());
            break;
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
            output = current_primitive_.direct_control();
            break;
        }
        case TbotsProto::Primitive::kMove:
        {
            if (!trajectory_path_.has_value() || !angular_trajectory_.has_value())
            {
                auto prim = createDirectControlPrimitive(Vector(), AngularVelocity(), 0.0,
                                                         TbotsProto::AutoChipOrKick());
                output    = prim->direct_control();
                LOG(INFO)
                    << "Not moving because trajectory_path_ or angular_trajectory_ is not set";
                setPrevCommandedVelocity(Vector(), AngularVelocity());
                break;
            }

            const Vector local_velocity = stepTargetLinearVelocity(delta_time_s);
            const AngularVelocity angular_velocity =
                stepTargetAngularVelocity(delta_time_s);

            // For debugging:
            sendLinearMotionToPlotJuggler(local_velocity, delta_time_s);

            auto prim = createDirectControlPrimitive(
                local_velocity, angular_velocity,
                convertDribblerModeToDribblerSpeed(
                    current_primitive_.move().dribbler_mode(), robot_constants_),
                current_primitive_.move().auto_chip_or_kick());

            output = prim->direct_control();
            break;
        }
        case TbotsProto::Primitive::PRIMITIVE_NOT_SET:
        {
            // TODO (#2283) Once we can add/remove robots, this log should
            // be re-enabled. Right now it just gets spammed because we command
            // 6 robots for Div B when there are 11 on the field.
            //
            // LOG(DEBUG) << "No primitive set!";
            break;
        }
        default:
        {
            prim_exec_status.set_running_primitive(false);
            setPrevCommandedVelocity(Vector(), AngularVelocity());
            output = TbotsProto::DirectControlPrimitive();
            break;
        }
    }

    robot_status.set_last_handled_primitive_seq_num(current_primitive_.sequence_number());

    const auto step_end    = std::chrono::steady_clock::now();
    using Millis           = std::chrono::duration<double, std::milli>;
    const Millis step_time = std::chrono::duration_cast<Millis>(step_end - step_start);

    robot_status.mutable_thunderloop_status()->set_primitive_executor_step_time_ms(
        step_time.count());

    return output;
}

void PrimitiveExecutor::setPrevCommandedVelocity(const Vector& local_velocity,
                                                 const AngularVelocity& angular_velocity)
{
    prev_target_global_velocity_ =
        localToGlobalVelocity(local_velocity, robot_state_.orientation());
    prev_target_angular_velocity_ = angular_velocity;
}

void PrimitiveExecutor::sendLinearMotionToPlotJuggler(const Vector& target_local_velocity,
                                                      const double delta_time_s) const
{
    const Vector& local_acceleration =
        (target_local_velocity -
         globalToLocalVelocity(robot_state_.velocity(), robot_state_.orientation())) /
        delta_time_s;

    const std::string robot_prefix = "robot_" + std::to_string(robot_id_);

    LOG(PLOTJUGGLER) << *createPlotJugglerValue(
        {{robot_prefix + "/x", robot_state_.position().x()},
         {robot_prefix + "/y", robot_state_.position().y()},
         {robot_prefix + "/v_x", robot_state_.velocity().x()},
         {robot_prefix + "/v_y", robot_state_.velocity().y()},
         {robot_prefix + "/target_v_x", target_local_velocity.x()},
         {robot_prefix + "/target_v_y", target_local_velocity.y()},
         {robot_prefix + "/target_a_x", local_acceleration.x()},
         {robot_prefix + "/target_a_y", local_acceleration.y()}});
}
