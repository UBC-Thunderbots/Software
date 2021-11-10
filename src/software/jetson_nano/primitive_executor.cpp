#include "software/jetson_nano/primitive_executor.h"

#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "software/logger/logger.h"
#include "software/math/math_functions.h"


void PrimitiveExecutor::startPrimitive(const RobotConstants_t& robot_constants,
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
        LOG(WARNING) << "Not currently executing a move primitive,"
                     << " cannot compute target velocity.";
        return Vector();
    }

    // Unpack current move primitive
    const float dest_linear_speed = current_primitive_.move().final_speed_m_per_s();
    const float max_speed_m_per_s = current_primitive_.move().max_speed_m_per_s();
    const Point final_position =
        Point(current_primitive_.move().destination().x_meters(),
              current_primitive_.move().destination().y_meters());
    std::clamp(max_speed_m_per_s, 0.0f, robot_constants_.robot_max_speed_m_per_s);

    const float max_target_linear_speed = fmaxf(max_speed_m_per_s, dest_linear_speed);

    // Compute distance to destination
    const float norm_dist_delta =
        static_cast<float>((robot_state.position() - final_position).length());

    // Compute at what linear distance we should start decelerating
    // d = (Vf^2 - Vi^2) / (2a + LOCAL_EPSILON)
    const float start_linear_deceleration_distance =
        (max_target_linear_speed * max_target_linear_speed -
         dest_linear_speed * dest_linear_speed) /
        (2 * robot_constants_.robot_max_acceleration_m_per_s_2 + LOCAL_EPSILON);

    // Create a sigmoid that is 1 when we are further than the
    // start_linear_deceleration_distance. It ~linearly goes from 1 to ~0 over
    // the deceleration distance. We shift the sigmoid by a factor of 4 so that
    // it doesn't fully zero out the speed before we are at the destination.
    const float target_linear_speed =
        max_target_linear_speed *
        static_cast<float>(sigmoid(norm_dist_delta,
                                   start_linear_deceleration_distance / 4,
                                   start_linear_deceleration_distance));

    Vector target_global_velocity = final_position - robot_state.position();

    double local_x_velocity =
        robot_state.orientation().cos() * target_global_velocity.x() +
        robot_state.orientation().sin() * target_global_velocity.y();

    double local_y_velocity =
        -robot_state.orientation().sin() * target_global_velocity.x() +
        robot_state.orientation().cos() * target_global_velocity.y();

    return Vector(local_x_velocity, local_y_velocity).normalize(target_linear_speed);
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity(const RobotState& robot_state)
{
    const float LOCAL_EPSILON = 1e-6f;  // Avoid dividing by zero

    if (current_primitive_.primitive_case() != TbotsProto::Primitive::kMove)
    {
        LOG(WARNING) << "Not a move primitive, cannot compute target velocity";
    }

    const float dest_orientation = current_primitive_.move().final_angle().radians();
    const float delta_orientation =
        dest_orientation - static_cast<float>(robot_state.orientation().toRadians());
    const float max_target_angular_speed = robot_constants_.robot_max_ang_speed_rad_per_s;

    // Compute at what angular distance we should start decelerating angularly
    // d = (Vf^2 - 0) / (2a + LOCAL_EPSILON)
    const float start_angular_deceleration_distance =
        (max_target_angular_speed * max_target_angular_speed) /
        (2 * robot_constants_.robot_max_ang_acceleration_rad_per_s_2 + LOCAL_EPSILON);

    const float target_angular_speed =
        max_target_angular_speed *
        static_cast<float>(sigmoid(fabsf(delta_orientation),
                                   start_angular_deceleration_distance / 2,
                                   start_angular_deceleration_distance));

    return AngularVelocity::fromRadians(
        copysign(target_angular_speed, delta_orientation));
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
            // TODO (#2283) Once we can add/remove robots, this log should
            // be re-enabled. Right now it just gets spammed because we command
            // 6 robots when there are 11 on the field.
            //
            // LOG(DEBUG) << "No primitive set!";
        }
    }
    return std::make_unique<TbotsProto::DirectControlPrimitive>();
}
