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
    : current_primitive_(), robot_constants_(robot_constants)
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
        (target_v_local - state_.localVelocity()) / delta_time.toSeconds();

    if (local_acceleration.length() > robot_constants_.robot_max_acceleration_m_per_s_2)
    {
        LOG(WARNING) << "Robot trying to accelerate at " << local_acceleration.length()
                     << "m/s^2.";
    }

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
        (target_w - state_.angularVelocity()) / delta_time.toSeconds();
    if (angular_acceleration.toRadians() >
        robot_constants_.robot_max_ang_acceleration_rad_per_s_2)
    {
        LOG(WARNING) << "Robot trying to angular accelerate at "
                     << angular_acceleration.toRadians() << "rads/s^2.";
    }
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
    return std::make_unique<TbotsProto::DirectControlPrimitive>();
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
