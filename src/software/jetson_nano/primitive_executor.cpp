#include "software/jetson_nano/primitive_executor.h"

#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "software/logger/logger.h"

extern "C"
{
#include "firmware/app/control/trajectory_planner.h"
}


void PrimitiveExecutor::startPrimitive(const RobotState& robot_state,
                                       const RobotConstants_t& robot_constants,
                                       const TbotsProto::Primitive& primitive)
{
    robot_constants_   = robot_constants;
    current_primitive_ = primitive;

    // Unpack Move Primitive
    Point final_destination = Point(current_primitive_.move().destination().x_meters(),
                                    current_primitive_.move().destination().y_meters());

    Angle final_angle =
        Angle::fromRadians(current_primitive_.move().final_angle().radians());

    float final_speed_m_per_s   = current_primitive_.move().final_speed_m_per_s();
    float target_spin_rev_per_s = current_primitive_.move().target_spin_rev_per_s();
    float max_speed_m_per_s     = current_primitive_.move().max_speed_m_per_s();
    std::clamp(max_speed_m_per_s, 0.0f, robot_constants_.robot_max_speed_m_per_s);

    // Compute displacement to destination
    Vector disp               = (robot_state.position() - final_destination);
    float dist_to_dest_length = static_cast<float>(disp.length());
    float dist_to_dest_x      = static_cast<float>(disp.x());
    float dist_to_dest_y      = static_cast<float>(disp.y());

    // Grab current robot position and orientation
    float robot_current_x     = static_cast<float>(robot_state.position().x());
    float robot_current_y     = static_cast<float>(robot_state.position().y());
    float robot_current_speed = static_cast<float>(robot_state.velocity().length());
    float robot_current_orientation =
        static_cast<float>(robot_state.orientation().toRadians());

    // Number of revolutions to spin, assuming the time horizon is the
    // simplistic dist_to_dest_length over max_speed_m_per_s
    const float revolutions_to_spin =
        (dist_to_dest_length / max_speed_m_per_s * target_spin_rev_per_s);

    // Change in orientation to reach destination orientation
    const float net_change_in_orientation =
        static_cast<float>(robot_state.orientation().minDiff(final_angle).toRadians());
    const float orientation_delta =
        net_change_in_orientation + revolutions_to_spin * 2.0f * (float)M_PI;

    const float estimated_time_delta = fmaxf(
        fabsf(dist_to_dest_length) / (float)(robot_constants.robot_max_speed_m_per_s),
        fabsf(net_change_in_orientation) /
            (float)(robot_constants_.robot_max_ang_speed_rad_per_s));


    // clamp num elements between 3 (minimum number of trajectory elements) and
    // TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
    num_elements_ = (unsigned int)fmaxf(
        fminf((estimated_time_delta * CONTROL_LOOP_HZ / NUM_TICKS_PER_TRAJECTORY_ELEMENT),
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
        .num_elements        = num_elements_,
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

Vector PrimitiveExecutor::getTargetLinearVelocity(const RobotState& robot_state)
{
    const float LOCAL_EPSILON = 1e-6f;  // Avoid dividing by zero

    if (current_primitive_.primitive_case() != TbotsProto::Primitive::kMove)
    {
        LOG(WARNING) << "Not a move primitive, cannot compute target velocity";
    }

    // Unpack current move primitive
    const float final_speed_m_per_s = current_primitive_.move().final_speed_m_per_s();
    const float max_speed_m_per_s   = current_primitive_.move().max_speed_m_per_s();
    const float dest_linear_speed =
        position_trajectory_.linear_speed[num_elements_ - 1];  // final speed
    const Point final_position =
        Point(current_primitive_.move().destination().x_meters(),
              current_primitive_.move().destination().y_meters());

    std::clamp(max_speed_m_per_s, 0.0f, robot_constants_.robot_max_speed_m_per_s);

    const float max_target_linear_speed = fmaxf(max_speed_m_per_s, final_speed_m_per_s);

    // Compute length of distance to destination
    const float norm_dist_delta =
        static_cast<float>((robot_state.position() - final_position).length());

    // Compute at what linear distance we should start decelerating
    // d = (Vf^2 - Vi^2) / (2a + LOCAL_EPSILON)
    const float start_linear_deceleration_distance =
        (max_target_linear_speed * max_target_linear_speed -
         dest_linear_speed * dest_linear_speed) /
        (2 * robot_constants_.robot_max_acceleration_m_per_s_2 + LOCAL_EPSILON);

    float target_linear_speed = max_target_linear_speed;
    if (norm_dist_delta < start_linear_deceleration_distance)
    {
        // Interpolate target speed between initial speed and final speed while the robot
        // is within start_linear_deceleration_distance away from the destination, also
        // add a minimum speed so the robot gets to the destination faster when dest speed
        // is 0
        target_linear_speed =
            fmaxf((max_target_linear_speed - dest_linear_speed) *
                          (norm_dist_delta /
                           (start_linear_deceleration_distance + LOCAL_EPSILON)) +
                      dest_linear_speed,
                  0.1f);
    }

    Vector target_velocity =
        (final_position - robot_state.position()).normalize(target_linear_speed);

    return target_velocity;
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity(const RobotState& robot_state)
{
    const float LOCAL_EPSILON = 1e-6f;  // Avoid dividing by zero

    if (current_primitive_.primitive_case() != TbotsProto::Primitive::kMove)
    {
        LOG(WARNING) << "Not a move primitive, cannot compute target velocity";
    }

    const float dest_orientation = position_trajectory_.orientation[num_elements_ - 1];
    const float dest_angular_speed =
        position_trajectory_.angular_speed[num_elements_ - 1];  // final speed
    const float delta_orientation =
        dest_orientation - static_cast<float>(robot_state.orientation().toRadians());
    const float max_target_angular_speed = robot_constants_.robot_max_ang_speed_rad_per_s;

    // Compute at what angular distance we should start decelerating angularly
    // d = (Vf^2 - Vi^2) / (2a + LOCAL_EPSILON)
    const float start_angular_deceleration_distance =
        (max_target_angular_speed * max_target_angular_speed -
         dest_angular_speed * dest_angular_speed) /
        (2 * robot_constants_.robot_max_ang_acceleration_rad_per_s_2 + LOCALrEPSILON);

    float target_angular_speed = max_target_angular_speed;

    if (fabsf(delta_orientation) < start_angular_deceleration_distance)
    {
        // Interpolate target angular speed between initial and final angular speed while
        // the robot is within start_angular_deceleration_distance away from the
        // destination, also add a minimum speed so the robot gets to the destination
        // faster when dest speed is 0
        target_angular_speed = fmaxf(
            (max_target_angular_speed - dest_angular_speed) *
                    (delta_orientation / (start_angular_deceleration_distance + 1e-6f)) +
                dest_angular_speed,
            0.01f * delta_orientation / (fabsf(delta_orientation) + 1e-6f));
    }

    return AngularVelocity::fromRadians(target_angular_speed);
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    const RobotState& robot_state)
{
    switch (current_primitive_.primitive_case())
    {
        case TbotsProto::Primitive::kEstop:
        {
            // Protobuf guarantees that the default values in a proto are all zero (bools
            // are false)
            //
            // https://developers.google.com/protocol-buffers/docs/proto3#default
            auto output = std::make_unique<TbotsProto::DirectControlPrimitive>();

            // Discharge the capacitors
            output->set_charge_mode(
                TbotsProto::DirectControlPrimitive_ChargeMode_DISCHARGE);

            return output;
        }
        case TbotsProto::Primitive::kStop:
        {
            auto prim   = createDirectControlPrimitive(Vector(), AngularVelocity(), 0.0);
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
            // Compute the target velocities
            Vector target_velocity = getTargetLinearVelocity(robot_state);
            AngularVelocity target_angular_velocity =
                getTargetAngularVelocity(robot_state);
            auto output = createDirectControlPrimitive(target_velocity,
                                                       target_angular_velocity, 0.0);
            return std::make_unique<TbotsProto::DirectControlPrimitive>(
                output->direct_control());
        }
        case TbotsProto::Primitive::PRIMITIVE_NOT_SET:
        {
        }
    }
    return std::make_unique<TbotsProto::DirectControlPrimitive>();
}
