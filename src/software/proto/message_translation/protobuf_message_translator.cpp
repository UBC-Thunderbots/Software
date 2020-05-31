#include "software/proto/message_translation/protobuf_message_translator.h"

#include <ctime>

#include "software/proto/message_translation/protobuf_primitive_visitor.h"

std::unique_ptr<VisionMsg> ProtobufMessageTranslator::getVisionMsgFromWorld(
    const World& world)
{
    auto vision_msg = std::make_unique<VisionMsg>();

    // get current timestamp
    auto timestamp = ProtobufMessageTranslator::getCurrentTimestampMsg();
    vision_msg->set_allocated_timestamp(timestamp.release());

    // get robot states
    auto& robot_states_map = *vision_msg->mutable_robot_states();
    auto friendly_robots   = world.friendlyTeam().getAllRobots();

    std::for_each(friendly_robots.begin(), friendly_robots.end(),
                  [&](const Robot& robot) {
                      robot_states_map[robot.id()] =
                          *ProtobufMessageTranslator::getRobotStateMsgFromRobot(robot);
                  });

    // get ball state
    auto ball_state = ProtobufMessageTranslator::getBallStateMsgFromBall(world.ball());
    vision_msg->set_allocated_ball_state(ball_state.release());

    return std::move(vision_msg);
}

std::unique_ptr<PrimitiveMsg>
ProtobufMessageTranslator::getPrimitiveMsgFromPrimitiveVector(
    const ConstPrimitiveVectorPtr& primitives)
{
    auto primitive_msg = std::make_unique<PrimitiveMsg>();

    // get current timestamp
    auto timestamp = ProtobufMessageTranslator::getCurrentTimestampMsg();
    primitive_msg->set_allocated_timestamp(timestamp.release());

    // get primitives
    auto& robot_primitives_map = *primitive_msg->mutable_robot_primitives();
    auto primitive_visitor     = ProtobufPrimitiveVisitor();

    std::for_each(primitives->begin(), primitives->end(),
                  [&](const std::unique_ptr<Primitive>& primitive) {
                      primitive->accept(primitive_visitor);
                      robot_primitives_map[primitive->getRobotId()] =
                          *primitive_visitor.getRadioPrimitiveMsg();
                  });


    return std::move(primitive_msg);
}

std::unique_ptr<RobotStateMsg> ProtobufMessageTranslator::getRobotStateMsgFromRobot(
    const Robot& robot)
{
    auto position = ProtobufMessageTranslator::getPointMsgFromPoint(robot.position());
    auto orientation =
        ProtobufMessageTranslator::getAngleMsgFromAngle(robot.orientation());
    auto velocity = ProtobufMessageTranslator::getVectorMsgFromVector(robot.velocity());
    auto angular_velocity =
        ProtobufMessageTranslator::getAngleMsgFromAngle(robot.angularVelocity());

    auto robot_state_msg = std::make_unique<RobotStateMsg>();

    robot_state_msg->set_allocated_global_position_meters(position.release());
    robot_state_msg->set_allocated_global_orientation_radians(orientation.release());
    robot_state_msg->set_allocated_global_velocity_meters_per_sec(velocity.release());
    robot_state_msg->set_allocated_global_angular_velocity_radians_per_sec(
        angular_velocity.release());

    return std::move(robot_state_msg);
}

std::unique_ptr<BallStateMsg> ProtobufMessageTranslator::getBallStateMsgFromBall(
    const Ball& ball)
{
    auto position = ProtobufMessageTranslator::getPointMsgFromPoint(ball.position());
    auto ball_state_msg = std::make_unique<BallStateMsg>();

    ball_state_msg->set_allocated_global_position_meters(position.release());

    return std::move(ball_state_msg);
}

std::unique_ptr<PointMsg> ProtobufMessageTranslator::getPointMsgFromPoint(
    const Point& point)
{
    auto point_msg = std::make_unique<PointMsg>();
    point_msg->set_x(point.x());
    point_msg->set_y(point.y());
    return std::move(point_msg);
}

std::unique_ptr<AngleMsg> ProtobufMessageTranslator::getAngleMsgFromAngle(
    const Angle& angle)
{
    auto angle_msg = std::make_unique<AngleMsg>();
    angle_msg->set_radians(angle.toRadians());
    return std::move(angle_msg);
}

std::unique_ptr<VectorMsg> ProtobufMessageTranslator::getVectorMsgFromVector(
    const Vector& vector)
{
    auto vector_msg = std::make_unique<VectorMsg>();
    vector_msg->set_x_component(vector.x());
    vector_msg->set_y_component(vector.y());
    return std::move(vector_msg);
}

std::unique_ptr<TimestampMsg> ProtobufMessageTranslator::getCurrentTimestampMsg()
{
    auto timestamp_msg = std::make_unique<TimestampMsg>();
    timestamp_msg->set_epoch_timestamp_seconds(std::time(0));
    return std::move(timestamp_msg);
}
