#include "software/proto/message_translation/protobuf_message_translation.h"

#include "shared/constants.h"
#include "shared/proto/geometry.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "shared/proto/vision.pb.h"
#include "software/primitive/primitive.h"
#include "software/proto/message_translation/protobuf_primitive_visitor.h"
#include "software/world/world.h"

std::unique_ptr<VisionMsg> convertWorldToVisionMsgProto(const World& world)
{
    // create msg and set timestamp
    auto vision_msg = std::make_unique<VisionMsg>();
    vision_msg->set_allocated_time_sent(getCurrentTimestampMsg().release());

    // set robot_states map
    auto& robot_states_map = *vision_msg->mutable_robot_states();
    auto friendly_robots   = world.friendlyTeam().getAllRobots();

    // For every friendly robot, we create a RobotStateMsg proto. The unique_ptr
    // is dereferenced, and there is an implicit deep copy into robot_states_map
    //
    // Since the unique_ptr immediately loses scope after the copy, the memory is
    // freed
    std::for_each(
        friendly_robots.begin(), friendly_robots.end(), [&](const Robot& robot) {
            robot_states_map[robot.id()] = *convertRobotToRobotStateMsgProto(robot);
        });

    // set ball state
    vision_msg->set_allocated_ball_state(
        convertBallToBallStateMsgProto(world.ball()).release());

    return std::move(vision_msg);
}

std::unique_ptr<PrimitiveMsg> convertPrimitiveVectortoPrimitiveMsgProto(
    const ConstPrimitiveVectorPtr& primitives)
{
    // create msg and update timestamp
    auto primitive_msg = std::make_unique<PrimitiveMsg>();
    primitive_msg->set_allocated_time_sent(getCurrentTimestampMsg().release());

    // set robot primitives
    auto& robot_primitives_map = *primitive_msg->mutable_robot_primitives();
    auto primitive_visitor     = ProtobufPrimitiveVisitor();

    // For every primitive that is converted, the unique_ptr is dereferenced,
    // and there is an implicit deep copy into the robot_primitives_map
    //
    // Since the unique_ptr immediately loses scope after the copy, the memory is
    // freed
    std::for_each(primitives->begin(), primitives->end(), [&](const auto& primitive) {
        robot_primitives_map[primitive->getRobotId()] =
            *primitive_visitor.getRadioPrimitiveMsg(*primitive);
    });

    return std::move(primitive_msg);
}

std::unique_ptr<RobotStateMsg> convertRobotToRobotStateMsgProto(const Robot& robot)
{
    auto position         = convertPointToPointMsgProto(robot.position());
    auto orientation      = convertAngleToAngleMsgProto(robot.orientation());
    auto velocity         = convertVectorToVectorMsgProto(robot.velocity());
    auto angular_velocity = convertAngleToAngleMsgProto(robot.angularVelocity());

    auto robot_state_msg = std::make_unique<RobotStateMsg>();

    robot_state_msg->set_allocated_global_position_meters(position.release());
    robot_state_msg->set_allocated_global_orientation_radians(orientation.release());
    robot_state_msg->set_allocated_global_velocity_meters_per_sec(velocity.release());
    robot_state_msg->set_allocated_global_angular_velocity_radians_per_sec(
        angular_velocity.release());

    return std::move(robot_state_msg);
}

std::unique_ptr<BallStateMsg> convertBallToBallStateMsgProto(const Ball& ball)
{
    auto position       = convertPointToPointMsgProto(ball.position());
    auto velocity       = convertVectorToVectorMsgProto(ball.velocity());
    auto ball_state_msg = std::make_unique<BallStateMsg>();

    ball_state_msg->set_allocated_global_position_meters(position.release());
    ball_state_msg->set_allocated_global_velocity_meters_per_sec(velocity.release());

    return std::move(ball_state_msg);
}

std::unique_ptr<PointMsg> convertPointToPointMsgProto(const Point& point)
{
    auto point_msg = std::make_unique<PointMsg>();
    point_msg->set_x(point.x());
    point_msg->set_y(point.y());
    return std::move(point_msg);
}

std::unique_ptr<AngleMsg> convertAngleToAngleMsgProto(const Angle& angle)
{
    auto angle_msg = std::make_unique<AngleMsg>();
    angle_msg->set_radians(angle.toRadians());
    return std::move(angle_msg);
}

std::unique_ptr<VectorMsg> convertVectorToVectorMsgProto(const Vector& vector)
{
    auto vector_msg = std::make_unique<VectorMsg>();
    vector_msg->set_x_component(vector.x());
    vector_msg->set_y_component(vector.y());
    return std::move(vector_msg);
}

std::unique_ptr<TimestampMsg> getCurrentTimestampMsg()
{
    auto timestamp_msg    = std::make_unique<TimestampMsg>();
    const auto clock_time = std::chrono::system_clock::now();
    double time_in_seconds =
        static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                clock_time.time_since_epoch())
                                .count()) /
        MICROSECONDS_PER_SECOND;

    timestamp_msg->set_epoch_timestamp_seconds(time_in_seconds);
    return std::move(timestamp_msg);
}
