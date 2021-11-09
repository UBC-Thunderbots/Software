#include "software/jetson_nano/velocity_trajectory_primitive_executor.h"

#include "proto/primitive.pb.h"

extern "C"
{
#include "firmware/app/control/trajectory_planner.h"
}

void VelocityTrajectoryPrimitiveExecutor::startPrimitive(
    RobotConstants_t robot_constants, RobotId robot_id, const World& world,
    std::unique_ptr<TbotsProto::Primitive> primitive)
{
    robot_constants_   = robot_constants;
    robot_id_          = robot_id;
    current_primitive_ = std::move(primitive);

    auto robot = world.friendlyTeam().getRobotById(robot_id);

    if (!robot.has_value())
    {
        // TODO Log and return
    }

    if (current_primitive_->primitive_case() == TbotsProto::Primitive::kMove)
    {
        // Unpack Move Primitive
        Point final_destination =
            Point(current_primitive_->move().destination().x_meters(),
                  current_primitive_->move().destination().y_meters());

        Angle final_angle =
            Angle::fromRadians(current_primitive_->move().final_angle().radians());

        float final_speed_m_per_s   = current_primitive_->move().final_speed_m_per_s();
        float target_spin_rev_per_s = current_primitive_->move().target_spin_rev_per_s();
        float max_speed_m_per_s     = current_primitive_->move().max_speed_m_per_s();
        std::clamp(max_speed_m_per_s, 0.0f, robot_constants_.robot_max_speed_m_per_s);

        // Compute displacement to destination
        Vector disp               = (robot->position() - final_destination);
        float dist_to_dest_length = static_cast<float>(disp.length());
        float dist_to_dest_x      = static_cast<float>(disp.x());
        float dist_to_dest_y      = static_cast<float>(disp.y());

        // Grab current robot position and orientation
        float robot_current_x     = static_cast<float>(robot->position().x());
        float robot_current_y     = static_cast<float>(robot->position().y());
        float robot_current_speed = static_cast<float>(robot->velocity().length());
        float robot_current_orientation =
            static_cast<float>(robot->orientation().toRadians());

        // Number of revolutions to spin, assuming the time horizon is the
        // simplistic dist_to_dest_length over max_speed_m_per_s
        const float revolutions_to_spin =
            (dist_to_dest_length / max_speed_m_per_s * target_spin_rev_per_s);

        // Change in orientation to reach destination orientation
        const float net_change_in_orientation =
            static_cast<float>(robot->orientation().minDiff(final_angle).toRadians());
        const float orientation_delta =
            net_change_in_orientation + revolutions_to_spin * 2.0f * (float)M_PI;

        const float estimated_time_delta = fmaxf(
            fabsf(dist_to_dest_length) / (float)(robot_constants.robot_max_speed_m_per_s),
            fabsf(net_change_in_orientation) /
                (float)(robot_constants_.robot_max_ang_speed_rad_per_s));


        // clamp num elements between 3 (minimum number of trajectory elements) and
        // TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
        const unsigned int num_elements =
            (unsigned int)fmaxf(fminf((estimated_time_delta * CONTROL_LOOP_HZ /
                                       NUM_TICKS_PER_TRAJECTORY_ELEMENT),
                                      TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS),
                                3);

        // Plan a trajectory to move to the target position/orientation
        FirmwareRobotPathParameters_t path_parameters = {
            .path = {.x = {.coefficients = {0, 0, dist_to_dest_x, robot_current_x}},
                     .y = {.coefficients = {0, 0, dist_to_dest_y, robot_current_y}}},
            .orientation_profile = {.coefficients = {0, 0, orientation_delta,
                                                     robot_current_orientation}},
            .t_start             = 0,
            .t_end               = 1.0f,
            .num_elements        = num_elements,
            .max_allowable_linear_acceleration =
                robot_constants_.robot_max_acceleration_m_per_s_2,
            .max_allowable_linear_speed = max_speed_m_per_s,
            .max_allowable_angular_acceleration =
                robot_constants_.robot_max_ang_acceleration_rad_per_s_2,
            .max_allowable_angular_speed = robot_constants_.robot_max_ang_speed_rad_per_s,
            .initial_linear_speed        = robot_current_speed,
            .final_linear_speed          = final_speed_m_per_s};

        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &position_trajectory_);
    }
}

std::unique_ptr<TbotsProto::DirectControlPrimitive_DirectVelocityControl>
VelocityTrajectoryPrimitiveExecutor::planVelocity(const Point& robot_current_position,
                                                  const Vector& robot_current_velocity,
                                                  const Angle& robot_current_orientation,
                                                  const Point& robot_target_position,
                                                  const float target_spin_rev_per_s,
                                                  const float max_speed_m_per_s,
                                                  const float final_speed_m_per_s)
{
    // const float dest_linear_speed = position_trajectory_.linear_speed[num_elements -
    // 1];

    // auto required_displacement = robot_target_position - robot_current_position;

    // const float max_target_linear_speed = fmaxf(max_speed_m_per_s,
    // final_speed_m_per_s); const float norm_dist_delta         =
    // required_displacement.length();

    // const float start_linear_deceleration_distance =
    //(max_target_linear_speed * max_target_linear_speed -
    // dest_linear_speed * dest_linear_speed) /
    //(2 * app_firmware_robot_getRobotConstants(robot)
    //.robot_max_acceleration_m_per_s_2 +
    // 1e-6f);

    // float target_linear_speed = max_target_linear_speed;
    // if (norm_dist_delta < start_linear_deceleration_distance)
    // {
    //// interpolate target speed between initial speed and final speed while the robot
    //// is within start_linear_deceleration_distance away from the destination, also
    //// add a minimum speed so the robot gets to the destination faster when dest speed
    //// is 0
    // target_linear_speed = fmaxf(
    //(max_target_linear_speed - dest_linear_speed) *
    //(norm_dist_delta / (start_linear_deceleration_distance + 1e-6f)) +
    // dest_linear_speed,
    // 0.1f);
    // }
    // float global_robot_velocity[2];
    // global_robot_velocity[0] = delta_x / (norm_dist_delta + 1e-6f) *
    // target_linear_speed; global_robot_velocity[1] = delta_y / (norm_dist_delta + 1e-6f)
    // * target_linear_speed;

    // float local_norm_vec[2][2] = {
    //{cosf(curr_orientation), sinf(curr_orientation)},
    //{cosf(curr_orientation + P_PI / 2), sinf(curr_orientation + P_PI / 2)}};

    // float local_robot_velocity[2];
    // for (int i = 0; i < 2; i++)
    // {
    //// interpolate target speed between initial speed and final speed while the robot
    //// is within start_linear_deceleration_distance away from the destination, also
    //// add a minimum speed so the robot gets to the final orientation faster when dest
    //// speed is 0
    // local_robot_velocity[i] =
    // shared_physics_dot2D(local_norm_vec[i], global_robot_velocity);
    // }


    // const float dest_orientation = pos_trajectory.orientation[num_elements - 1];
    // const float dest_angular_speed =
    // pos_trajectory.angular_speed[num_elements - 1];  // final speed
    // const float delta_orientation = dest_orientation - curr_orientation;
    // const float max_angular_speed =
    // app_firmware_robot_getRobotConstants(robot).robot_max_ang_speed_rad_per_s;
    // const float max_target_angular_speed =
    // fmaxf(max_angular_speed, dest_angular_speed);  // initial speed
    // const float start_angular_deceleration_distance =
    //(max_target_angular_speed * max_target_angular_speed -
    // dest_angular_speed * dest_angular_speed) /
    //(2 * app_firmware_robot_getRobotConstants(robot)
    //.robot_max_ang_acceleration_rad_per_s_2 +
    // 1e-6f);
    // float target_angular_speed = max_target_angular_speed;
    // if (fabsf(delta_orientation) < start_angular_deceleration_distance)
    // {
    // target_angular_speed = fmaxf(
    //(max_target_angular_speed - dest_angular_speed) *
    //(delta_orientation / (start_angular_deceleration_distance + 1e-6f)) +
    // dest_angular_speed,
    // 0.01f * delta_orientation / (fabsf(delta_orientation) + 1e-6f));
    // }

    // TbotsProto_DirectControlPrimitive_DirectVelocityControl control_msg;
    // control_msg.velocity.x_component_meters         = local_robot_velocity[0];
    // control_msg.velocity.y_component_meters         = local_robot_velocity[1];
    // control_msg.angular_velocity.radians_per_second = target_angular_speed;

    // velocity_wheels_setLocalVelocity(robot, control_msg);

    return std::make_unique<TbotsProto::DirectControlPrimitive_DirectVelocityControl>();
}

std::unique_ptr<TbotsProto::DirectControlPrimitive>
VelocityTrajectoryPrimitiveExecutor::stepPrimitive(const World& world)
{
    switch (current_primitive_->primitive_case())
    {
        case TbotsProto::Primitive::kEstop:
        {
        }
        case TbotsProto::Primitive::kStop:
        {
        }
        case TbotsProto::Primitive::kDirectControl:
        {
            return std::make_unique<TbotsProto::DirectControlPrimitive>(
                current_primitive_->direct_control());
        }
        case TbotsProto::Primitive::kMove:
        {
        }
        case TbotsProto::Primitive::PRIMITIVE_NOT_SET:
        {
        }
    }
    return std::make_unique<TbotsProto::DirectControlPrimitive>();
}
