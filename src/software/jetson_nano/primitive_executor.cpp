#include "software/jetson_nano/primitive_executor.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/logger/logger.h"
#include "software/math/math_functions.h"

PrimitiveExecutor::PrimitiveExecutor(const double time_step, const RobotId robot_id,
                                     const RobotConstants_t &robot_constants,
                                     const TeamColour friendly_team_colour)
    : robot_id_(robot_id),
      current_primitive_(),
      hrvo_simulator_(static_cast<float>(time_step), robot_constants,
                      friendly_team_colour)
{
    time_step_ = time_step;
    curr_orientation_ = Angle::zero();
}

void PrimitiveExecutor::updatePrimitiveSet(
    const unsigned int robot_id, const TbotsProto::PrimitiveSet& primitive_set_msg)
{
    hrvo_simulator_.updatePrimitiveSet(primitive_set_msg);
    auto primitive_set_msg_iter = primitive_set_msg.robot_primitives().find(robot_id);
    if (primitive_set_msg_iter != primitive_set_msg.robot_primitives().end())
    {
        current_primitive_ = primitive_set_msg_iter->second;
        return;
    }
}

void PrimitiveExecutor::clearCurrentPrimitive()
{
    current_primitive_.Clear();
}

void PrimitiveExecutor::updateWorld(const TbotsProto::World& world_msg)
{
//    current_world_ = world_msg;
    const World world = World(world_msg);
    auto this_robot = world.friendlyTeam().getRobotById(robot_id_);
    if (this_robot.has_value())
    {
        curr_orientation_ = this_robot.value().orientation();
    }
    hrvo_simulator_.updateWorld(world);
}

void PrimitiveExecutor::updateLocalVelocity(const Vector &local_velocity,
                                            const Angle &curr_orientation)
{
    hrvo_simulator_.updateFriendlyRobotVelocity(robot_id_,
                                                local_velocity.rotate(-curr_orientation));
}

Vector PrimitiveExecutor::getTargetLinearVelocity(const Angle &curr_orientation)
{
    Vector target_global_velocity = hrvo_simulator_.getRobotVelocity(robot_id_);
    return target_global_velocity.rotate(-curr_orientation);
//    return Vector(0, 0.04);
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity(
    const TbotsProto::MovePrimitive& move_primitive, const Angle& curr_orientation)
{
    const Angle dest_orientation = createAngle(move_primitive.final_angle());
    const double delta_orientation =
        dest_orientation.minDiff(curr_orientation).toRadians();

    double deceleration_angular_speed = std::sqrt(
        2 * move_primitive.robot_max_ang_acceleration_rad_per_s_2() * delta_orientation);

    double max_angular_speed =
        static_cast<double>(move_primitive.robot_max_ang_speed_rad_per_s());

    double next_angular_speed = std::min(max_angular_speed, deceleration_angular_speed);

    const double signed_delta_orientation =
        (dest_orientation - curr_orientation).clamp().toRadians();
    return AngularVelocity::fromRadians(
        std::copysign(next_angular_speed, signed_delta_orientation));
}

double PrimitiveExecutor::getTargetLinearSpeed(
    const TbotsProto::MovePrimitive& move_primitive, const RobotState& robot_state)
{
    const float LOCAL_EPSILON = 1e-6f;  // Avoid dividing by zero

    // Unpack current move primitive
    const float dest_linear_speed = move_primitive.final_speed_m_per_s();
    const float max_speed_m_per_s = move_primitive.max_speed_m_per_s();
    const Point final_position =
        createPoint(move_primitive.motion_control().path().points().at(1));

    const float max_target_linear_speed = fmaxf(max_speed_m_per_s, dest_linear_speed);

    // Compute distance to destination
    const float norm_dist_delta =
        static_cast<float>((robot_state.position() - final_position).length());

    // Compute at what linear distance we should start decelerating
    // d = (Vf^2 - Vi^2) / (2a + LOCAL_EPSILON)
    const float start_linear_deceleration_distance =
        (max_target_linear_speed * max_target_linear_speed -
         dest_linear_speed * dest_linear_speed) /
        (2 * move_primitive.robot_max_acceleration_m_per_s_2() + LOCAL_EPSILON);

    // When we are close enough to start decelerating, we reduce the max speed
    // by 60%. Once we get closer than 0.6 meters, we start to linearly decrease
    // speed proportional to the distance to the destination. 0.6 was determined
    // experimentally.
    float target_linear_speed = max_target_linear_speed;
    if (norm_dist_delta < start_linear_deceleration_distance)
    {
        target_linear_speed = max_target_linear_speed * fminf(norm_dist_delta, 0.6f);
    }

    return target_linear_speed;
}

Vector PrimitiveExecutor::getTargetLinearVelocity(
    const TbotsProto::MovePrimitive& move_primitive, const RobotState& robot_state)
{
    const float LOCAL_EPSILON = 1e-6f;  // Avoid dividing by zero

    // Unpack current move primitive
    const float dest_linear_speed = move_primitive.final_speed_m_per_s();
    const float max_speed_m_per_s = move_primitive.max_speed_m_per_s();
    const Point final_position =
        createPoint(move_primitive.motion_control().path().points().at(1));

    const float max_target_linear_speed = fmaxf(max_speed_m_per_s, dest_linear_speed);

    // Compute distance to destination
    const float norm_dist_delta =
        static_cast<float>((robot_state.position() - final_position).length());

    // Compute at what linear distance we should start decelerating
    // d = (Vf^2 - Vi^2) / (2a + LOCAL_EPSILON)
    const float start_linear_deceleration_distance =
        (max_target_linear_speed * max_target_linear_speed -
         dest_linear_speed * dest_linear_speed) /
        (2 * move_primitive.robot_max_acceleration_m_per_s_2() + LOCAL_EPSILON);

//    (max_target_linear_speed * max_target_linear_speed) /  // Changes formula here and removed dest_linear_speed * dest_linear_speed
//    (2 * move_primitive.robot_max_acceleration_m_per_s_2() + LOCAL_EPSILON)

    // When we are close enough to start decelerating, we reduce the max speed
    // by 60%. Once we get closer than 0.6 meters, we start to linearly decrease
    // speed proportional to the distance to the destination. 0.6 was determined
    // experimentally.
    float target_linear_speed = max_target_linear_speed;
    if (norm_dist_delta < start_linear_deceleration_distance)
    {
        target_linear_speed = max_target_linear_speed * fminf(norm_dist_delta, 0.6f);
    }

    Vector target_global_velocity = final_position - robot_state.position();

    double local_x_velocity =
        robot_state.orientation().cos() * target_global_velocity.x() +
        robot_state.orientation().sin() * target_global_velocity.y();

    double local_y_velocity =
        -robot_state.orientation().sin() * target_global_velocity.x() +
        robot_state.orientation().cos() * target_global_velocity.y();

    return Vector(local_x_velocity, local_y_velocity).normalize(target_linear_speed);
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    const unsigned int robot_id, const RobotState& robot_state)
{
    hrvo_simulator_.doStep();

    // Visualize the HRVO Simulator for the current robot
    robot_id_ = robot_id;
    hrvo_simulator_.visualize(robot_id);

    switch (current_primitive_.primitive_case())
    {
        case TbotsProto::Primitive::kEstop:
        {
            // Protobuf guarantees that the default values in a proto are all zero (bools
            // are false)
            //
            // https://developers.google.com/protocol-buffers/docs/proto3#default
            auto output = std::make_unique<TbotsProto::DirectControlPrimitive>();

            return output;
        }
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
            // Compute the target velocities
            // Vector target_velocity = getTargetLinearVelocity(current_primitive_.move(),
            // robot_state);
//            Vector target_velocity = getTargetLinearVelocity(robot_state.orientation());
            Vector target_velocity = getTargetLinearVelocity(current_primitive_.move(), robot_state);
//            Vector target_velocity = getTargetLinearVelocity(curr_orientation_);

//            double target_linear_speed =
//                getTargetLinearSpeed(current_primitive_.move(), robot_state);
//            if (target_velocity.length() > target_linear_speed)
//            {
//                target_velocity = target_velocity.normalize(target_linear_speed);
//            }

            AngularVelocity target_angular_velocity = getTargetAngularVelocity(
                current_primitive_.move(), curr_orientation_);

            curr_orientation_ += target_angular_velocity * time_step_;

            auto output = createDirectControlPrimitive(
                target_velocity, target_angular_velocity,
                current_primitive_.move().dribbler_speed_rpm(),
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
