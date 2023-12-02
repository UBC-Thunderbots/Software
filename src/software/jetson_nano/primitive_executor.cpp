#include "software/jetson_nano/primitive_executor.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/physics/velocity_conversion_util.h"

PrimitiveExecutor::PrimitiveExecutor(const Duration time_step,
                                     const RobotConstants_t &robot_constants,
                                     const TeamColour friendly_team_colour,
                                     const RobotId robot_id)
    : current_primitive_(),
      current_world_(),
      friendly_team_colour_(friendly_team_colour),
      robot_constants_(robot_constants),
      hrvo_simulator_(robot_id),
      time_step_(time_step),
      robot_id_(robot_id)
{
}

void PrimitiveExecutor::updatePrimitiveSet(
    const TbotsProto::PrimitiveSet &primitive_set_msg)
{
    hrvo_simulator_.updatePrimitiveSet(primitive_set_msg, time_step_);
    auto primitive_set_msg_iter = primitive_set_msg.robot_primitives().find(robot_id_);
    if (primitive_set_msg_iter != primitive_set_msg.robot_primitives().end())
    {
        current_primitive_ = primitive_set_msg_iter->second;

        if (current_primitive_.has_move())
        {
            const TbotsProto::MovePrimitive &move_traj = current_primitive_.move();
            const TbotsProto::TrajectoryPathParams2D &trajectory_2d_params =
                move_traj.xy_traj_params();
            const TbotsProto::TrajectoryParamsAngular1D &trajectory_angular_params =
                move_traj.w_traj_params();

            trajectory_path_ = createTrajectoryPathFromParams(
                trajectory_2d_params, robot_constants_, velocity_);

            // TODO: Combine generate and constructor
            angular_trajectory_ = BangBangTrajectory1DAngular();
            angular_trajectory_->generate(
                createAngle(trajectory_angular_params.start_angle()),
                createAngle(trajectory_angular_params.final_angle()),
                angular_velocity_,  // createAngularVelocity(trajectory_angular_params.initial_velocity()),
                AngularVelocity::fromRadians(
                    robot_constants_.robot_max_ang_speed_rad_per_s),
                AngularVelocity::fromRadians(
                    robot_constants_.robot_max_ang_acceleration_rad_per_s_2),
                AngularVelocity::fromRadians(
                    robot_constants_.robot_max_ang_acceleration_rad_per_s_2));

            time_since_trajectory_creation_ =
                Duration::fromSeconds(VISION_TO_ROBOT_DELAY_S);
        }
        return;
    }
}

void PrimitiveExecutor::setStopPrimitive()
{
    current_primitive_ = *createStopPrimitiveProto();
}

void PrimitiveExecutor::updateWorld(const TbotsProto::World &world_msg)
{
    // Only update HRVO simulator if the world is newer than the previous world
    if (world_msg.time_sent().epoch_timestamp_seconds() >
        current_world_.time_sent().epoch_timestamp_seconds())
    {
        hrvo_simulator_.updateWorld(World(world_msg), robot_constants_, time_step_);
    }
}

void PrimitiveExecutor::updateVelocity(const Vector &local_velocity,
                                       const AngularVelocity &angular_velocity)
{
    // To allow robots to accelerate smoothly, we only update their simulated velocity if
    // it is significantly different from the actual robot velocity

    std::optional<Angle> orientation_opt = hrvo_simulator_.getRobotOrientation(robot_id_);
    if (!orientation_opt.has_value())
    {
        return;
    }

    Vector curr_hrvo_velocity     = hrvo_simulator_.getRobotVelocity(robot_id_);
    Vector actual_global_velocity = localToGlobalVelocity(local_velocity, orientation_);
    velocity_                     = actual_global_velocity;
    if ((curr_hrvo_velocity - actual_global_velocity).length() >
        LINEAR_VELOCITY_FEEDBACK_THRESHOLD_M_PER_S)
    {
        hrvo_simulator_.updateRobotVelocity(robot_id_, actual_global_velocity);
    }

    AngularVelocity curr_angular_velocity =
        hrvo_simulator_.getRobotAngularVelocity(robot_id_);
    angular_velocity_ = angular_velocity;
    if (angular_velocity.minDiff(curr_angular_velocity).toDegrees() >
        ANGULAR_VELOCITY_FEEDBACK_THRESHOLD_DEG_PER_S)
    {
        hrvo_simulator_.updateRobotAngularVelocity(robot_id_, angular_velocity);
    }
}

Vector PrimitiveExecutor::getTargetLinearVelocity()
{
    std::optional<Angle> orientation_opt = hrvo_simulator_.getRobotOrientation(robot_id_);
    if (orientation_opt.has_value())
    {
        Vector target_global_velocity = hrvo_simulator_.getRobotVelocity(robot_id_);
        return globalToLocalVelocity(target_global_velocity, orientation_opt.value());
    }
    else
    {
        return Vector();
    }
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity()
{
    return hrvo_simulator_.getRobotAngularVelocity(robot_id_);
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive()
{
    hrvo_simulator_.doStep(time_step_);

    // Visualize the HRVO Simulator for the current robot
    //    hrvo_simulator_.visualize(robot_id_, friendly_team_colour_);

    switch (current_primitive_.primitive_case())
    {
        case TbotsProto::Primitive::kStop:
        {
            auto prim   = createDirectControlPrimitive(Vector(), AngularVelocity(), 0.0,
                                                     TbotsProto::AutoChipOrKick());
            auto output = std::make_unique<TbotsProto::DirectControlPrimitive>(
                prim->direct_control());
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
                LOG(DEBUG)
                    << "Not moving because trajectory_path_ or angular_trajectory_ is not set";
                return output;
            }

            time_since_trajectory_creation_ += time_step_;
            if (robot_id_ == 4 && friendly_team_colour_ == TeamColour::BLUE)
            {
                auto desired_vel = trajectory_path_->getVelocity(
                    time_since_trajectory_creation_.toSeconds());
                LOG(PLOTJUGGLER) << *createPlotJugglerValue({
                    {"actual_vx", velocity_.x()},
                    {"actual_vy", velocity_.y()},
                    {"actual_v", velocity_.length()},
                    {"desired_vx", desired_vel.x()},
                    {"desired_vy", desired_vel.y()},
                    {"desired_v", desired_vel.length()},
                });
            }

            TbotsProto::HRVOVisualization hrvo_visualization;
            *(hrvo_visualization.add_robots()) = *createCircleProto(
                Circle(trajectory_path_->getPosition(0), ROBOT_MAX_RADIUS_METERS));
            hrvo_visualization.set_robot_id(robot_id_);
            TbotsProto::Path path_proto;
            const int num_points = 18;
            for (int j = 0; j <= num_points; ++j)
            {
                Point pos = trajectory_path_->getPosition(
                    j * trajectory_path_->getTotalTime() / num_points);
                *(path_proto.add_points()) = *createPointProto(pos);
            }
            *(hrvo_visualization.mutable_trajectory()) = path_proto;

            if (friendly_team_colour_ == TeamColour::YELLOW)
            {
                LOG(VISUALIZE, YELLOW_HRVO_PATH) << hrvo_visualization;
            }
            else
            {
                LOG(VISUALIZE, BLUE_HRVO_PATH) << hrvo_visualization;
            }

            // TODO: Notes
            //  By default, the robot is very tweaky/shaky around destination
            //  This helps, even with local velocity feedback its twitchy
            orientation_ = angular_trajectory_->getPosition(
                time_since_trajectory_creation_.toSeconds());
            Vector local_velocity =
                globalToLocalVelocity(trajectory_path_->getVelocity(
                                          time_since_trajectory_creation_.toSeconds()),
                                      orientation_);
            Point position = trajectory_path_->getPosition(
                time_since_trajectory_creation_.toSeconds());
            double distance_to_destination =
                distance(position, trajectory_path_->getDestination());
            if (distance_to_destination < 0.05)
            {
                local_velocity *= distance_to_destination / 0.05;
            }

            AngularVelocity angular_velocity = angular_trajectory_->getVelocity(
                time_since_trajectory_creation_.toSeconds());
            Angle orientation_to_destination =
                orientation_.minDiff(angular_trajectory_->getDestination());
            if (orientation_to_destination.toDegrees() < 5)
            {
                angular_velocity *= orientation_to_destination.toDegrees() / 5;
            }

            LOG(PLOTJUGGLER) << *createPlotJugglerValue(
                {{"orientation_", orientation_.toRadians()},
                 {"v", local_velocity.length()},
                 {"vx", local_velocity.x()},
                 {"vy", local_velocity.y()},
                 {"local_v", velocity_.length()},
                 {"local_vx", velocity_.x()},
                 {"local_vy", velocity_.y()},
                 {"px", position.x()},
                 {"py", position.y()},
                 {"d_to_dest", distance_to_destination},
                 {"actual_vel_desired_vel", (local_velocity - velocity_).length()},
                 {"actual_vel_desired_vel_x", (local_velocity - velocity_).x()},
                 {"actual_vel_desired_vel_y", (local_velocity - velocity_).y()},
                 {"vt", angular_velocity.toRadians()},
                 {"dt", orientation_to_destination.toRadians()}});

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

void PrimitiveExecutor::setRobotId(const RobotId robot_id)
{
    robot_id_ = robot_id;
}
