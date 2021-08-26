#include "software/simulation/er_force_simulator.h"

#include "software/proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "software/proto/message_translation/ssl_detection.h"
#include "software/proto/message_translation/ssl_geometry.h"
#include "software/proto/message_translation/ssl_wrapper.h"
#include "software/simulation/er_force_simulator_robot_singleton.h"
#include "software/simulation/simulator_ball_singleton.h"

extern "C"
{
#include "firmware/app/logger/logger.h"
#include "firmware/app/world/firmware_ball.h"
#include "firmware/app/world/firmware_robot.h"
#include "firmware/app/world/firmware_world.h"
#include "shared/proto/robot_log_msg.nanopb.h"
}

ErForceSimulator::ErForceSimulator(
    const Field& field, std::shared_ptr<const SimulatorConfig> simulator_config,
    const Duration& physics_time_step)
    : physics_world(field, simulator_config),
      yellow_team_defending_side(FieldSide::NEG_X),
      blue_team_defending_side(FieldSide::NEG_X),
      frame_number(0),
      physics_time_step(physics_time_step),
      er_force_sim_timer(),
      er_force_sim_setup(),
      er_force_sim(&er_force_sim_timer, er_force_sim_setup)
{
    this->resetCurrentFirmwareTime();
}

void ErForceSimulator::setBallState(const BallState& ball_state)
{
    er_force_sim.safelyTeleportBall(ball_state.position().x(), ball_state.position().y());
}

void ErForceSimulator::addYellowRobots(const std::vector<RobotStateWithId>& robots)
{
    // TODO: add robots
}

void ErForceSimulator::addBlueRobots(const std::vector<RobotStateWithId>& robots)
{
    // TODO: add robots
}

void ErForceSimulator::setYellowRobotPrimitive(RobotId id,
                                               const TbotsProto_Primitive& primitive_msg,
                                               const TbotsProto_Vision& vision_msg)
{
    setRobotPrimitive(id, primitive_msg, yellow_simulator_robots, simulator_ball,
                      yellow_team_defending_side);
}

void ErForceSimulator::setBlueRobotPrimitive(RobotId id,
                                             const TbotsProto_Primitive& primitive_msg,
                                             const TbotsProto_Vision& vision_msg)
{
    setRobotPrimitive(id, primitive_msg, blue_simulator_robots, simulator_ball,
                      blue_team_defending_side);
}

void ErForceSimulator::setYellowRobotPrimitiveSet(
    const TbotsProto_PrimitiveSet& primitive_set_msg, const TbotsProto_Vision& vision_msg)
{
    for (pb_size_t i = 0; i < primitive_set_msg.robot_primitives_count; i++)
    {
        setYellowRobotPrimitive(primitive_set_msg.robot_primitives[i].key,
                                primitive_set_msg.robot_primitives[i].value, vision_msg);
    }
}

void ErForceSimulator::setBlueRobotPrimitiveSet(
    const TbotsProto_PrimitiveSet& primitive_set_msg, const TbotsProto_Vision& vision_msg)
{
    for (pb_size_t i = 0; i < primitive_set_msg.robot_primitives_count; i++)
    {
        setBlueRobotPrimitive(primitive_set_msg.robot_primitives[i].key,
                              primitive_set_msg.robot_primitives[i].value, vision_msg);
    }
}

void ErForceSimulator::setRobotPrimitive(
    RobotId id, const TbotsProto_Primitive& primitive_msg,
    std::map<std::shared_ptr<ErForceSimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>&
        simulator_robots,
    const std::shared_ptr<ErForceSimulatorBall>& simulator_ball, FieldSide defending_side)
{
    // TODO: make sure simulator_ball is updated with the right BallState
    SimulatorBallSingleton::setSimulatorBall(simulator_ball, defending_side);
    auto simulator_robots_iter =
        std::find_if(simulator_robots.begin(), simulator_robots.end(),
                     [id](const auto& robot_world_pair) {
                         return robot_world_pair.first->getRobotId() == id;
                     });

    if (simulator_robots_iter != simulator_robots.end())
    {
        auto simulator_robot = (*simulator_robots_iter).first;
        auto firmware_world  = (*simulator_robots_iter).second;
        // TODO: make sure simulator_robot is updated with the right RobotState
        ErForceSimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
        ErForceSimulatorRobotSingleton::startNewPrimitiveOnCurrentSimulatorRobot(
            firmware_world, primitive_msg);
    }
}

void ErForceSimulator::stepSimulation(const Duration& time_step)
{
    // Set the ball being referenced in each firmware_world.
    // We only need to do this a single time since all robots
    // can see and interact with the same ball

    Duration remaining_time = time_step;
    while (remaining_time > Duration::fromSeconds(0))
    {
        current_firmware_time = physics_world.getTimestamp();

        for (auto& iter : blue_simulator_robots)
        {
            auto simulator_robot = iter.first;
            auto firmware_world  = iter.second;

            app_logger_init(simulator_robot->getRobotId(),
                            &ErForceSimulatorRobotSingleton::handleBlueRobotLogProto);

            // TODO: make sure simulator_robot is updated with the right RobotState
            ErForceSimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
            // TODO: make sure simulator_ball is updated with the right BallState
            SimulatorBallSingleton::setSimulatorBall(simulator_ball, FieldSide::NEG_X);
            ErForceSimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(
                firmware_world);
        }

        for (auto& iter : yellow_simulator_robots)
        {
            auto simulator_robot = iter.first;
            auto firmware_world  = iter.second;

            app_logger_init(simulator_robot->getRobotId(),
                            &ErForceSimulatorRobotSingleton::handleYellowRobotLogProto);

            // TODO: make sure simulator_robot is updated with the right RobotState
            ErForceSimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
            // TODO: make sure simulator_ball is updated with the right BallState
            SimulatorBallSingleton::setSimulatorBall(simulator_ball, FieldSide::NEG_X);
            ErForceSimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(
                firmware_world);
        }

        // We take as many steps of `physics_time_step` as possible, and then
        // simulate the remainder of the time
        // TODO: replace this
        // Duration dt = std::min(remaining_time, physics_time_step);
        // physics_world.stepSimulation(dt);
        // remaining_time = remaining_time - physics_time_step;
    }

    frame_number++;
}

std::unique_ptr<SSLProto::SSL_WrapperPacket> ErForceSimulator::getSSLWrapperPacket() const
{
    auto ball_state  = physics_world.getBallState();
    auto ball_states = ball_state.has_value()
                           ? std::vector<BallState>({ball_state.value()})
                           : std::vector<BallState>();
    auto detection_frame = createSSLDetectionFrame(
        CAMERA_ID, physics_world.getTimestamp(), frame_number, ball_states,
        physics_world.getYellowRobotStates(), physics_world.getBlueRobotStates());
    auto geometry_data =
        createGeometryData(physics_world.getField(), FIELD_LINE_THICKNESS_METRES);
    auto wrapper_packet =
        createSSLWrapperPacket(std::move(geometry_data), std::move(detection_frame));
    return wrapper_packet;
}

Field ErForceSimulator::getField() const
{
    return physics_world.getField();
}

Timestamp ErForceSimulator::getTimestamp() const
{
    return physics_world.getTimestamp();
}

void ErForceSimulator::addYellowRobot(const Point& position)
{
    // TODO: implement
}

void ErForceSimulator::addBlueRobot(const Point& position)
{
    // TODO: implement
}

void ErForceSimulator::resetCurrentFirmwareTime()
{
    current_firmware_time = Timestamp::fromSeconds(0);
}

float ErForceSimulator::getCurrentFirmwareTimeSeconds()
{
    return static_cast<float>(current_firmware_time.toSeconds());
}

// We must give this variable a value here, as non-const static variables must be
// initialized out-of-line
Timestamp ErForceSimulator::current_firmware_time = Timestamp::fromSeconds(0);

// float io_vision_getBallPositionX(void)
//{
//    io_lock_vision();
//    float temp = vision.ball_state.global_position.x_meters;
//    io_unlock_vision();
//    return temp;
//}
// float io_vision_getBallPositionY(void)
//{
//    io_lock_vision();
//    float temp = vision.ball_state.global_position.y_meters;
//    io_unlock_vision();
//    return temp;
//}
// float io_vision_getBallVelocityX(void)
//{
//    io_lock_vision();
//    float temp = vision.ball_state.global_velocity.x_component_meters;
//    io_unlock_vision();
//    return temp;
//}
// float io_vision_getBallVelocityY(void)
//{
//    io_lock_vision();
//    float temp = vision.ball_state.global_velocity.y_component_meters;
//    io_unlock_vision();
//    return temp;
//}
// float io_vision_getRobotPositionX(void)
//{
//    float temp = 0.0f;
//    io_lock_vision();
//            if (vision.robot_states_count == 1)
//            {
//             temp = vision.robot_states[0].value.global_position.x_meters;
//            }
//    io_unlock_vision();
//    return temp;
//}
// float io_vision_getRobotPositionY(void)
//{
//    float temp = 0.0f;
//    io_lock_vision();
//            if (vision.robot_states_count == 1)
//            {
//             temp = vision.robot_states[0].value.global_position.y_meters;
//            }
//    io_unlock_vision();
//    return temp;
//}
// float io_vision_getRobotOrientation(void)
//{
//    float temp = 0.0f;
//    io_lock_vision();
//            if (vision.robot_states_count == 1)
//            {
//             temp = vision.robot_states[0].value.global_orientation.radians;
//            }
//    io_unlock_vision();
//    return temp;
//}
// float io_vision_getRobotVelocityX(void)
//{
//    float temp = 0.0f;
//    io_lock_vision();
//            if (vision.robot_states_count == 1)
//            {
//             temp = vision.robot_states[0].value.global_velocity.x_component_meters;
//            }
//    io_unlock_vision();
//    return temp;
//}
// float io_vision_getRobotVelocityY(void)
//{
//    float temp = 0.0f;
//    io_lock_vision();
//            if (vision.robot_states_count == 1)
//            {
//             temp = vision.robot_states[0].value.global_velocity.y_component_meters;
//            }
//    io_unlock_vision();
//    return temp;
//}
// float io_vision_getRobotAngularVelocity(void)
//{
//    float temp = 0.0f;
//    io_lock_vision();
//            if (vision.robot_states_count == 1)
//            {
//             temp =
//             vision.robot_states[0].value.global_angular_velocity.radians_per_second;
//            }
//    io_unlock_vision();
//    return temp;
//}
