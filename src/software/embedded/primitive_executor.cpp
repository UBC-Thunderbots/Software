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
        const auto new_trajectory_path = createTrajectoryPathFromParams(
            current_primitive_.move().xy_traj_params(), state_.position(),
            state_.velocity(), robot_constants_);

        const auto new_angular_trajectory = createAngularTrajectoryFromParams(
            current_primitive_.move().w_traj_params(), state_.orientation(),
            state_.angularVelocity(), robot_constants_);

        const bool is_linear_trajectory_new =
            new_trajectory_path.has_value() != trajectory_path_.has_value() ||
            (trajectory_path_.has_value() &&
             !trajectory_path_->hasSameDestination(*new_trajectory_path,
                                                   LINEAR_DESTINATION_THRESHOLD_METERS));

        const bool is_angular_trajectory_new =
            !angular_trajectory_.has_value() ||
            !angular_trajectory_->hasSameDestination(
                new_angular_trajectory, ANGULAR_DESTINATION_THRESHOLD_DEGREES);

        LOG(PLOTJUGGLER) << *createPlotJugglerValue({{"new_trajectory", 0}});
        if (is_linear_trajectory_new || true)
        {
            LOG(PLOTJUGGLER) << *createPlotJugglerValue({{"new_trajectory", 1}});
            trajectory_path_ = new_trajectory_path;
            position_controller_.reset();
            time_since_linear_trajectory_creation_ =
                Duration::fromSeconds(VISION_TO_ROBOT_DELAY_S);
        }

        if (is_angular_trajectory_new || true)
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
    LOG(PLOTJUGGLER) << *createPlotJugglerValue({
        {"target_velocity_x",
         trajectory_path_->getVelocity(time_since_linear_trajectory_creation_.toSeconds())
             .x()},
        {"target_velocity_y",
         trajectory_path_->getVelocity(time_since_linear_trajectory_creation_.toSeconds())
             .y()},
        {"actual_velocity_x", state_.velocity().x()},
        {"actual_velocity_y", state_.velocity().y()},
    });
    const auto target_v_global =
        position_controller_.step(state_.position(), *trajectory_path_,
                                  time_since_linear_trajectory_creation_, delta_time);
    const auto target_v_local =
        globalToLocalVelocity(target_v_global, state_.orientation());

    return target_v_local.normalize(
        std::min(target_v_local.length(),
                 static_cast<double>(robot_constants_.robot_max_speed_m_per_s)));
}

AngularVelocity PrimitiveExecutor::stepTargetAngularVelocity(Duration delta_time)
{
    const auto target_w =
        orientation_controller_.step(state_.orientation(), *angular_trajectory_,
                                     time_since_angular_trajectory_creation_, delta_time);
    if (target_w.toRadians() < -robot_constants_.robot_max_ang_speed_rad_per_s)
    {
        return AngularVelocity::fromRadians(
            -robot_constants_.robot_max_ang_speed_rad_per_s);
    }
    if (target_w.toRadians() > robot_constants_.robot_max_ang_speed_rad_per_s)
    {
        return AngularVelocity::fromRadians(
            robot_constants_.robot_max_ang_speed_rad_per_s);
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
