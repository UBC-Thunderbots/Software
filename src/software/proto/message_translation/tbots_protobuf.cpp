#include "software/proto/message_translation/tbots_protobuf.h"

#include "software/proto/message_translation/proto_creator_primitive_visitor.h"

std::unique_ptr<TbotsProto::Vision> createVision(const World& world)
{
    // create msg and set timestamp
    auto vision_msg                    = std::make_unique<TbotsProto::Vision>();
    *(vision_msg->mutable_time_sent()) = *createCurrentTimestamp();

    // set robot_states map
    auto& robot_states_map = *vision_msg->mutable_robot_states();
    auto friendly_robots   = world.friendlyTeam().getAllRobots();

    // For every friendly robot, we create a RobotState proto. The unique_ptr
    // is dereferenced, and there is an implicit deep copy into robot_states_map
    //
    // Since the unique_ptr immediately loses scope after the copy, the memory is
    // freed
    std::for_each(friendly_robots.begin(), friendly_robots.end(),
                  [&](const Robot& robot) {
                      robot_states_map[robot.id()] = *createRobotState(robot);
                  });

    // set ball state
    *(vision_msg->mutable_ball_state()) = *createBallState(world.ball());

    return std::move(vision_msg);
}

std::unique_ptr<TbotsProto::PrimitiveSet> createPrimitiveSet(
    const ConstPrimitiveVectorPtr& primitives)
{
    // create msg and update timestamp
    auto primitive_set_msg = std::make_unique<TbotsProto::PrimitiveSet>();
    *(primitive_set_msg->mutable_time_sent()) = *createCurrentTimestamp();

    // set robot primitives
    auto& robot_primitives_map = *primitive_set_msg->mutable_robot_primitives();
    auto primitive_visitor     = ProtoCreatorPrimitiveVisitor();

    // For every primitive that is converted, the unique_ptr is dereferenced,
    // and there is an implicit deep copy into the robot_primitives_map
    //
    // Since the unique_ptr immediately loses scope after the copy, the memory is
    // freed
    std::for_each(primitives->begin(), primitives->end(), [&](const auto& primitive) {
        robot_primitives_map[primitive->getRobotId()] =
            primitive_visitor.createPrimitive(*primitive);
    });

    return std::move(primitive_set_msg);
}

std::unique_ptr<TbotsProto::RobotState> createRobotState(const Robot& robot)
{
    auto position         = createPoint(robot.position());
    auto orientation      = createAngle(robot.orientation());
    auto velocity         = createVector(robot.velocity());
    auto angular_velocity = createAngularVelocity(robot.angularVelocity());

    auto robot_state_msg = std::make_unique<TbotsProto::RobotState>();

    *(robot_state_msg->mutable_global_position())         = *position;
    *(robot_state_msg->mutable_global_orientation())      = *orientation;
    *(robot_state_msg->mutable_global_velocity())         = *velocity;
    *(robot_state_msg->mutable_global_angular_velocity()) = *angular_velocity;

    return std::move(robot_state_msg);
}

std::unique_ptr<TbotsProto::BallState> createBallState(const Ball& ball)
{
    auto position       = createPoint(ball.position());
    auto velocity       = createVector(ball.velocity());
    auto ball_state_msg = std::make_unique<TbotsProto::BallState>();

    *(ball_state_msg->mutable_global_position()) = *position;
    *(ball_state_msg->mutable_global_velocity()) = *velocity;

    return std::move(ball_state_msg);
}

std::unique_ptr<TbotsProto::Timestamp> createCurrentTimestamp()
{
    auto timestamp_msg    = std::make_unique<TbotsProto::Timestamp>();
    const auto clock_time = std::chrono::system_clock::now();
    double time_in_seconds =
        static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                clock_time.time_since_epoch())
                                .count()) /
        MICROSECONDS_PER_SECOND;

    timestamp_msg->set_epoch_timestamp_seconds(time_in_seconds);
    return std::move(timestamp_msg);
}
