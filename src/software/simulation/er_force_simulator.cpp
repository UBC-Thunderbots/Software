#include "software/simulation/er_force_simulator.h"

#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>

#include <QtCore/QFile>
#include <QtCore/QString>
#include <iostream>

#include "extlibs/er_force_sim/src/protobuf/robot.h"
#include "proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "proto/message_translation/ssl_detection.h"
#include "proto/message_translation/ssl_geometry.h"
#include "proto/message_translation/ssl_wrapper.h"
#include "software/simulation/er_force_simulator_robot_singleton.h"
#include "software/simulation/simulator_ball_singleton.h"

extern "C"
{
#include "firmware/app/logger/logger.h"
#include "firmware/app/world/firmware_ball.h"
#include "firmware/app/world/firmware_robot.h"
#include "firmware/app/world/firmware_world.h"
#include "proto/robot_log_msg.nanopb.h"
}

inline bool loadConfiguration(const QString& configFile,
                              google::protobuf::Message* message, bool allowPartial)
{
    QString fullFilename = QString("software/simulation/config/") + configFile + ".txt";
    QFile file(fullFilename);
    if (!file.open(QFile::ReadOnly))
    {
        std::cout << "Could not open configuration file " << fullFilename.toStdString()
                  << std::endl;
        return false;
    }
    QString str = file.readAll();
    file.close();
    std::string s = qPrintable(str);

    google::protobuf::TextFormat::Parser parser;
    parser.AllowPartialMessage(allowPartial);
    parser.ParseFromString(s, message);
    return true;
}

ErForceSimulator::~ErForceSimulator()
{
    delete er_force_sim;
}

ErForceSimulator::ErForceSimulator(
    const Field& field, const RobotConstants_t& robot_constants,
    const WheelConstants& wheel_constants,
    std::shared_ptr<const SimulatorConfig> simulator_config,
    const Duration& physics_time_step)
    : physics_world(field, robot_constants, wheel_constants, simulator_config),
      yellow_team_vision_msg(),
      blue_team_vision_msg(),
      frame_number(0),
      physics_time_step(physics_time_step),
      er_force_sim_timer(),
      //      er_force_sim_setup(),
      //      er_force_sim(&er_force_sim_timer, er_force_sim_setup, true),
      wrapper_packet()
{
    loadConfiguration("simulator/2020", &er_force_sim_setup, false);
    er_force_sim =
        new camun::simulator::Simulator(&er_force_sim_timer, er_force_sim_setup, true),

    // er_force_sim_timer.setTime(1234, 1.0);
        std::cout << "er_force_sim_timer.currentTime(): "
                  << er_force_sim_timer.currentTime() << std::endl;
    Command c{new amun::Command};
    c->mutable_simulator()->set_enable(true);
    // start with default robots, take ER-Force specs.
    robot::Specs ERForce;
    robotSetDefault(&ERForce);

    Team friendly_team = Team();
    Team enemy_team    = Team();
    Ball ball          = Ball(Point(), Vector(), Timestamp::fromSeconds(0));

    World world    = World(field, ball, friendly_team, enemy_team);
    wrapper_packet = *createSSLWrapperPacket(world, TeamColour::YELLOW);

    auto* teamBlue   = c->mutable_set_team_blue();
    auto* teamYellow = c->mutable_set_team_yellow();
    for (auto* team : {teamBlue, teamYellow})
    {
        for (int i = 0; i < 11; ++i)
        {
            auto* robot = team->add_robot();
            robot->CopyFrom(ERForce);
            robot->set_id(i);
        }
    }
    er_force_sim->handleCommand(c);

    QObject::connect(er_force_sim, &camun::simulator::Simulator::gotPacket, this,
                     &ErForceSimulator::setWrapperPacket);
    this->resetCurrentFirmwareTime();

    er_force_sim->process();
    // er_force_sim_timer.setTime(1235, 1.0);
    //    er_force_sim->process();
}

void ErForceSimulator::setBallState(const BallState& ball_state)
{
    er_force_sim->safelyTeleportBall(ball_state.position().x(),
                                     ball_state.position().y());
}

void ErForceSimulator::addYellowRobots(const std::vector<RobotStateWithId>& robots) {}

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
    simulator_ball = std::make_shared<ErForceSimulatorBall>(BallState(
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
        simulator_ball = std::make_shared<ErForceSimulatorBall>(BallState(
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
        simulator_ball = std::make_shared<ErForceSimulatorBall>(BallState(
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

    er_force_sim->handleRadioCommands(yellow_robot_control, false,
                                      er_force_sim_timer.currentTime());
    // er_force_sim->process();
    er_force_sim->handleRadioCommands(blue_robot_control, true,
                                      er_force_sim_timer.currentTime());
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(time_step.toMilliseconds())));
    er_force_sim->process();

    frame_number++;
}

std::unique_ptr<SSLProto::SSL_WrapperPacket> ErForceSimulator::getSSLWrapperPacket() const
{
    return std::make_unique<SSLProto::SSL_WrapperPacket>(wrapper_packet);
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

void ErForceSimulator::setWrapperPacket(const QByteArray& data, qint64 time,
                                        QString sender)
{
    auto packet_data = SSLProto::SSL_WrapperPacket();
    packet_data.ParseFromArray(data.data(), data.size());
    std::cout << "ErForceSimulator packet_data.detection().t_capture()"
              << packet_data.detection().t_capture() << std::endl;
    std::cout << "ErForceSimulator packet_data.detection().robots_blue_size()"
              << packet_data.detection().robots_blue_size() << std::endl;
    std::cout << "ErForceSimulator packet_data.detection().robots_yellow_size()"
              << packet_data.detection().robots_yellow_size() << std::endl;
    std::cout << "ErForceSimulator packet_data.detection().t_capture() rem"
              << static_cast<int>(packet_data.detection().t_capture()) % 100 << std::endl;
    wrapper_packet = packet_data;
    std::cout << "ErForceSimulator wrapper_packet.detection().t_capture()"
              << wrapper_packet.detection().t_capture() << std::endl;
    std::cout << "ErForceSimulator wrapper_packet.detection().t_capture() rem"
              << static_cast<int>(wrapper_packet.detection().t_capture()) % 100
              << std::endl;
}

// We must give this variable a value here, as non-const static variables must be
// initialized out-of-line
Timestamp ErForceSimulator::current_firmware_time = Timestamp::fromSeconds(0);
