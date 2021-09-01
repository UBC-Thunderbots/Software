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
      yellow_team_vision_msg(),
      blue_team_vision_msg(),
      frame_number(0),
      physics_time_step(physics_time_step),
      er_force_sim_timer(),
      er_force_sim_setup(),
      er_force_sim(&er_force_sim_timer, er_force_sim_setup, true)
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
                                               const TbotsProto_Primitive& primitive_msg)
{
    setRobotPrimitive(id, primitive_msg, yellow_simulator_robots, simulator_ball,
                      yellow_team_vision_msg);
}

void ErForceSimulator::setBlueRobotPrimitive(RobotId id,
                                             const TbotsProto_Primitive& primitive_msg)
{
    setRobotPrimitive(id, primitive_msg, blue_simulator_robots, simulator_ball,
                      blue_team_vision_msg);
}

void ErForceSimulator::setYellowRobotPrimitiveSet(
    const TbotsProto_PrimitiveSet& primitive_set_msg,
    const TbotsProto::Vision& vision_msg)
{
    for (pb_size_t i = 0; i < primitive_set_msg.robot_primitives_count; i++)
    {
        setYellowRobotPrimitive(primitive_set_msg.robot_primitives[i].key,
                                primitive_set_msg.robot_primitives[i].value);
    }
    yellow_team_vision_msg = vision_msg;
}

void ErForceSimulator::setBlueRobotPrimitiveSet(
    const TbotsProto_PrimitiveSet& primitive_set_msg,
    const TbotsProto::Vision& vision_msg)
{
    for (pb_size_t i = 0; i < primitive_set_msg.robot_primitives_count; i++)
    {
        setBlueRobotPrimitive(primitive_set_msg.robot_primitives[i].key,
                              primitive_set_msg.robot_primitives[i].value);
    }
    blue_team_vision_msg = vision_msg;
}

void ErForceSimulator::setRobotPrimitive(
    RobotId id, const TbotsProto_Primitive& primitive_msg,
    std::map<std::shared_ptr<ErForceSimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>&
        simulator_robots,
    std::shared_ptr<ErForceSimulatorBall> simulator_ball,
    const TbotsProto::Vision& vision_msg)
{
    simulator_ball->setState(BallState(
        Point(vision_msg.ball_state().global_position().x_meters(),
              vision_msg.ball_state().global_position().y_meters()),
        Vector(vision_msg.ball_state().global_velocity().x_component_meters(),
               vision_msg.ball_state().global_velocity().x_component_meters())));
    SimulatorBallSingleton::setSimulatorBall(simulator_ball, FieldSide::NEG_X);
    auto simulator_robots_iter =
        std::find_if(simulator_robots.begin(), simulator_robots.end(),
                     [id](const auto& robot_world_pair) {
                         return robot_world_pair.first->getRobotId() == id;
                     });

    if (simulator_robots_iter != simulator_robots.end())
    {
        auto simulator_robot = (*simulator_robots_iter).first;
        auto firmware_world  = (*simulator_robots_iter).second;
        simulator_robot->setRobotState(RobotState(
            Point(vision_msg.robot_states().at(id).global_position().x_meters(),
                  vision_msg.robot_states().at(id).global_position().y_meters()),
            Vector(
                vision_msg.robot_states().at(id).global_velocity().x_component_meters(),
                vision_msg.robot_states().at(id).global_velocity().x_component_meters()),
            Angle::fromRadians(
                vision_msg.robot_states().at(id).global_orientation().radians()),
            AngularVelocity::fromRadians(vision_msg.robot_states()
                                             .at(id)
                                             .global_angular_velocity()
                                             .radians_per_second())));
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

    current_firmware_time = physics_world.getTimestamp();

    SSLSimRobotControl yellow_robot_control{new sslsim::RobotControl};
    SSLSimRobotControl blue_robot_control{new sslsim::RobotControl};

    for (auto& iter : blue_simulator_robots)
    {
        auto simulator_robot = iter.first;
        auto firmware_world  = iter.second;

        app_logger_init(simulator_robot->getRobotId(),
                        &ErForceSimulatorRobotSingleton::handleBlueRobotLogProto);

        simulator_robot->setRobotState(
            RobotState(Point(blue_team_vision_msg.robot_states()
                                 .at(simulator_robot->getRobotId())
                                 .global_position()
                                 .x_meters(),
                             blue_team_vision_msg.robot_states()
                                 .at(simulator_robot->getRobotId())
                                 .global_position()
                                 .y_meters()),
                       Vector(blue_team_vision_msg.robot_states()
                                  .at(simulator_robot->getRobotId())
                                  .global_velocity()
                                  .x_component_meters(),
                              blue_team_vision_msg.robot_states()
                                  .at(simulator_robot->getRobotId())
                                  .global_velocity()
                                  .x_component_meters()),
                       Angle::fromRadians(blue_team_vision_msg.robot_states()
                                              .at(simulator_robot->getRobotId())
                                              .global_orientation()
                                              .radians()),
                       AngularVelocity::fromRadians(blue_team_vision_msg.robot_states()
                                                        .at(simulator_robot->getRobotId())
                                                        .global_angular_velocity()
                                                        .radians_per_second())));
        ErForceSimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
        simulator_ball->setState(BallState(
            Point(blue_team_vision_msg.ball_state().global_position().x_meters(),
                  blue_team_vision_msg.ball_state().global_position().y_meters()),
            Vector(
                blue_team_vision_msg.ball_state().global_velocity().x_component_meters(),
                blue_team_vision_msg.ball_state()
                    .global_velocity()
                    .x_component_meters())));
        SimulatorBallSingleton::setSimulatorBall(simulator_ball, FieldSide::NEG_X);
        ErForceSimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(
            firmware_world);
        *(blue_robot_control->mutable_robot_commands()->Add()) =
            *(simulator_robot->getRobotCommand());
    }

    for (auto& iter : yellow_simulator_robots)
    {
        auto simulator_robot = iter.first;
        auto firmware_world  = iter.second;

        app_logger_init(simulator_robot->getRobotId(),
                        &ErForceSimulatorRobotSingleton::handleYellowRobotLogProto);

        simulator_robot->setRobotState(
            RobotState(Point(yellow_team_vision_msg.robot_states()
                                 .at(simulator_robot->getRobotId())
                                 .global_position()
                                 .x_meters(),
                             yellow_team_vision_msg.robot_states()
                                 .at(simulator_robot->getRobotId())
                                 .global_position()
                                 .y_meters()),
                       Vector(yellow_team_vision_msg.robot_states()
                                  .at(simulator_robot->getRobotId())
                                  .global_velocity()
                                  .x_component_meters(),
                              yellow_team_vision_msg.robot_states()
                                  .at(simulator_robot->getRobotId())
                                  .global_velocity()
                                  .x_component_meters()),
                       Angle::fromRadians(yellow_team_vision_msg.robot_states()
                                              .at(simulator_robot->getRobotId())
                                              .global_orientation()
                                              .radians()),
                       AngularVelocity::fromRadians(yellow_team_vision_msg.robot_states()
                                                        .at(simulator_robot->getRobotId())
                                                        .global_angular_velocity()
                                                        .radians_per_second())));
        ErForceSimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
        simulator_ball->setState(BallState(
            Point(yellow_team_vision_msg.ball_state().global_position().x_meters(),
                  yellow_team_vision_msg.ball_state().global_position().y_meters()),
            Vector(yellow_team_vision_msg.ball_state()
                       .global_velocity()
                       .x_component_meters(),
                   yellow_team_vision_msg.ball_state()
                       .global_velocity()
                       .x_component_meters())));
        SimulatorBallSingleton::setSimulatorBall(simulator_ball, FieldSide::NEG_X);
        ErForceSimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(
            firmware_world);

        *(yellow_robot_control->mutable_robot_commands()->Add()) =
            *(simulator_robot->getRobotCommand());
    }

    er_force_sim.handleRadioCommands(yellow_robot_control, false,
                                     er_force_sim_timer.currentTime());
    er_force_sim.handleRadioCommands(blue_robot_control, true,
                                     er_force_sim_timer.currentTime());
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(time_step.toMilliseconds())));
    er_force_sim.process();

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
