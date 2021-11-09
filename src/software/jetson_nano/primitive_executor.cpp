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
}

Vector PrimitiveExecutor::getTargetLinearVelocity(const RobotState& robot_state)
{
    const float LOCAL_EPSILON = 1e-6f;  // Avoid dividing by zero

    if (current_primitive_.primitive_case() != TbotsProto::Primitive::kMove)
    {
        LOG(WARNING) << "Not a move primitive, cannot compute target velocity";
    }

    // Unpack current move primitive
    const float dest_linear_speed   = current_primitive_.move().final_speed_m_per_s();
    const float max_speed_m_per_s   = current_primitive_.move().max_speed_m_per_s();
    const Point final_position =
        Point(current_primitive_.move().destination().x_meters(),
              current_primitive_.move().destination().y_meters());
    std::clamp(max_speed_m_per_s, 0.0f, robot_constants_.robot_max_speed_m_per_s);

    const float max_target_linear_speed = fmaxf(max_speed_m_per_s, dest_linear_speed);

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

    const float dest_orientation   = current_primitive_.move().final_angle().radians();
    const float dest_angular_speed = 0.0f;  // No support for spin prim
    const float delta_orientation =
        dest_orientation - static_cast<float>(robot_state.orientation().toRadians());
    const float max_target_angular_speed = robot_constants_.robot_max_ang_speed_rad_per_s;

    // Compute at what angular distance we should start decelerating angularly
    // d = (Vf^2 - Vi^2) / (2a + LOCAL_EPSILON)
    const float start_angular_deceleration_distance =
        (max_target_angular_speed * max_target_angular_speed -
         dest_angular_speed * dest_angular_speed) /
        (2 * robot_constants_.robot_max_ang_acceleration_rad_per_s_2 + LOCAL_EPSILON);

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
            // LOG(DEBUG) << "No primitive set!";
        }
    }
    return std::make_unique<TbotsProto::DirectControlPrimitive>();
}
