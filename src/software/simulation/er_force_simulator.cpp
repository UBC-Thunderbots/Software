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
#include "proto/message_translation/ssl_simulation_robot_control.h"
#include "proto/message_translation/ssl_wrapper.h"
#include "software/simulation/er_force_simulator_robot_singleton.h"
#include "software/simulation/simulator_ball_singleton.h"
#include "software/world/robot_state.h"

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
    QString fullFilename =
        QString(
            "/home/jonathan/robocup/thunderbots/Software/src/software/simulation/config/") +
        configFile + ".txt";
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
      yellow_team_vision_msg(std::make_unique<TbotsProto::Vision>()),
      blue_team_vision_msg(std::make_unique<TbotsProto::Vision>()),
      frame_number(0),
      physics_time_step(physics_time_step),
      er_force_sim_timer(),
      robot_constants(robot_constants),
      wheel_constants(wheel_constants)
//      er_force_sim_setup(),
//      er_force_sim(&er_force_sim_timer, er_force_sim_setup, true),
{
    loadConfiguration("simulator/2020", &er_force_sim_setup, false);
    er_force_sim = new camun::simulator::Simulator(er_force_sim_setup, true),

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

    World world = World(field, ball, friendly_team, enemy_team);

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
    er_force_sim->handleSimulatorSetupCommand(c);

    for (unsigned int i = 0; i < 11; i++)
    {
        auto blue_simulator_robot = std::make_shared<ErForceSimulatorRobot>(
            RobotStateWithId{.id          = i,
                             .robot_state = RobotState(Point(), Vector(), Angle::zero(),
                                                       AngularVelocity::zero())},
            robot_constants, wheel_constants);
        auto yellow_simulator_robot = std::make_shared<ErForceSimulatorRobot>(
            RobotStateWithId{.id          = i,
                             .robot_state = RobotState(Point(), Vector(), Angle::zero(),
                                                       AngularVelocity::zero())},
            robot_constants, wheel_constants);

        auto blue_firmware_robot = ErForceSimulatorRobotSingleton::createFirmwareRobot();
        auto blue_firmware_ball  = SimulatorBallSingleton::createFirmwareBall();
        auto yellow_firmware_robot =
            ErForceSimulatorRobotSingleton::createFirmwareRobot();
        auto yellow_firmware_ball = SimulatorBallSingleton::createFirmwareBall();

        FirmwareWorld_t* blue_firmware_world_raw = app_firmware_world_create(
            blue_firmware_robot.release(), blue_firmware_ball.release(),
            &(ErForceSimulator::getCurrentFirmwareTimeSeconds));
        auto blue_firmware_world = std::shared_ptr<FirmwareWorld_t>(
            blue_firmware_world_raw, FirmwareWorldDeleter());

        FirmwareWorld_t* yellow_firmware_world_raw = app_firmware_world_create(
            yellow_firmware_robot.release(), blue_firmware_ball.release(),
            &(ErForceSimulator::getCurrentFirmwareTimeSeconds));
        auto yellow_firmware_world = std::shared_ptr<FirmwareWorld_t>(
            yellow_firmware_world_raw, FirmwareWorldDeleter());

        blue_simulator_robots.insert(
            std::make_pair(blue_simulator_robot, blue_firmware_world));
        yellow_simulator_robots.insert(
            std::make_pair(yellow_simulator_robot, yellow_firmware_world));
    }


    this->resetCurrentFirmwareTime();
    er_force_sim->stepSimulation(1);
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
    std::cout << "setYellowRobotPrimitive: yellow_team_vision_msg = "
              << yellow_team_vision_msg->DebugString() << std::endl;
    simulator_ball = std::make_shared<ErForceSimulatorBall>(BallState(
        Point(yellow_team_vision_msg->ball_state().global_position().x_meters(),
              yellow_team_vision_msg->ball_state().global_position().y_meters()),
        Vector(
            yellow_team_vision_msg->ball_state().global_velocity().x_component_meters(),
            yellow_team_vision_msg->ball_state()
                .global_velocity()
                .x_component_meters())));
    SimulatorBallSingleton::setSimulatorBall(simulator_ball, FieldSide::NEG_X);
    auto yellow_simulator_robots_iter =
        std::find_if(yellow_simulator_robots.begin(), yellow_simulator_robots.end(),
                     [id](const auto& robot_world_pair) {
                         return robot_world_pair.first->getRobotId() == id;
                     });

    if (yellow_simulator_robots_iter != yellow_simulator_robots.end())
    {
        auto simulator_robot = (*yellow_simulator_robots_iter).first;
        auto firmware_world  = (*yellow_simulator_robots_iter).second;

        auto robot_state_it = yellow_team_vision_msg->robot_states().find(id);
        if (robot_state_it == yellow_team_vision_msg->robot_states().end())
        {
            LOG(WARNING) << "setRobotPrimitive: Robot state for robot "
                         << simulator_robot->getRobotId() << " not found";
        }
        else
        {
            simulator_robot->setRobotState(RobotState(
                Point(yellow_team_vision_msg->robot_states()
                          .at(id)
                          .global_position()
                          .x_meters(),
                      yellow_team_vision_msg->robot_states()
                          .at(id)
                          .global_position()
                          .y_meters()),
                Vector(yellow_team_vision_msg->robot_states()
                           .at(id)
                           .global_velocity()
                           .x_component_meters(),
                       yellow_team_vision_msg->robot_states()
                           .at(id)
                           .global_velocity()
                           .x_component_meters()),
                Angle::fromRadians(yellow_team_vision_msg->robot_states()
                                       .at(id)
                                       .global_orientation()
                                       .radians()),
                AngularVelocity::fromRadians(yellow_team_vision_msg->robot_states()
                                                 .at(id)
                                                 .global_angular_velocity()
                                                 .radians_per_second())));
            ErForceSimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
            ErForceSimulatorRobotSingleton::startNewPrimitiveOnCurrentSimulatorRobot(
                firmware_world, primitive_msg);
        }
    }
    else
    {
        LOG(WARNING) << "Simulator robot with ID " << id << " not found" << std::endl;
    }
}

void ErForceSimulator::setBlueRobotPrimitive(RobotId id,
                                             const TbotsProto_Primitive& primitive_msg)
{
    std::cout << "setBlueRobotPrimitive: blue_team_vision_msg = "
              << blue_team_vision_msg->DebugString() << std::endl;
    simulator_ball = std::make_shared<ErForceSimulatorBall>(BallState(
        Point(blue_team_vision_msg->ball_state().global_position().x_meters(),
              blue_team_vision_msg->ball_state().global_position().y_meters()),
        Vector(
            blue_team_vision_msg->ball_state().global_velocity().x_component_meters(),
            blue_team_vision_msg->ball_state().global_velocity().x_component_meters())));
    SimulatorBallSingleton::setSimulatorBall(simulator_ball, FieldSide::NEG_X);
    auto blue_simulator_robots_iter =
        std::find_if(blue_simulator_robots.begin(), blue_simulator_robots.end(),
                     [id](const auto& robot_world_pair) {
                         return robot_world_pair.first->getRobotId() == id;
                     });

    if (blue_simulator_robots_iter != blue_simulator_robots.end())
    {
        auto simulator_robot = (*blue_simulator_robots_iter).first;
        auto firmware_world  = (*blue_simulator_robots_iter).second;

        auto robot_state_it = blue_team_vision_msg->robot_states().find(id);
        if (robot_state_it == blue_team_vision_msg->robot_states().end())
        {
            LOG(WARNING) << "setRobotPrimitive: Robot state for robot "
                         << simulator_robot->getRobotId() << " not found";
        }
        else
        {
            simulator_robot->setRobotState(RobotState(
                Point(blue_team_vision_msg->robot_states()
                          .at(id)
                          .global_position()
                          .x_meters(),
                      blue_team_vision_msg->robot_states()
                          .at(id)
                          .global_position()
                          .y_meters()),
                Vector(blue_team_vision_msg->robot_states()
                           .at(id)
                           .global_velocity()
                           .x_component_meters(),
                       blue_team_vision_msg->robot_states()
                           .at(id)
                           .global_velocity()
                           .x_component_meters()),
                Angle::fromRadians(blue_team_vision_msg->robot_states()
                                       .at(id)
                                       .global_orientation()
                                       .radians()),
                AngularVelocity::fromRadians(blue_team_vision_msg->robot_states()
                                                 .at(id)
                                                 .global_angular_velocity()
                                                 .radians_per_second())));
            ErForceSimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
            ErForceSimulatorRobotSingleton::startNewPrimitiveOnCurrentSimulatorRobot(
                firmware_world, primitive_msg);
        }
    }
    else
    {
        LOG(WARNING) << "Simulator robot with ID " << id << " not found" << std::endl;
    }
}

void ErForceSimulator::setYellowRobotPrimitiveSet(
    const TbotsProto_PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::Vision> vision_msg)
{
    for (pb_size_t i = 0; i < primitive_set_msg.robot_primitives_count; i++)
    {
        setYellowRobotPrimitive(primitive_set_msg.robot_primitives[i].key,
                                primitive_set_msg.robot_primitives[i].value);
    }
    yellow_team_vision_msg = std::move(vision_msg);
}

void ErForceSimulator::setBlueRobotPrimitiveSet(
    const TbotsProto_PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::Vision> vision_msg)
{
    for (pb_size_t i = 0; i < primitive_set_msg.robot_primitives_count; i++)
    {
        setBlueRobotPrimitive(primitive_set_msg.robot_primitives[i].key,
                              primitive_set_msg.robot_primitives[i].value);
    }
    blue_team_vision_msg = std::move(vision_msg);
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

        auto robot_state_it = vision_msg.robot_states().find(id);
        if (robot_state_it == vision_msg.robot_states().end())
        {
            LOG(WARNING) << "setRobotPrimitive: Robot state for robot "
                         << simulator_robot->getRobotId() << " not found";
        }
        else
        {
            simulator_robot->setRobotState(RobotState(
                Point(vision_msg.robot_states().at(id).global_position().x_meters(),
                      vision_msg.robot_states().at(id).global_position().y_meters()),
                Vector(vision_msg.robot_states()
                           .at(id)
                           .global_velocity()
                           .x_component_meters(),
                       vision_msg.robot_states()
                           .at(id)
                           .global_velocity()
                           .x_component_meters()),
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
    else
    {
        LOG(WARNING) << "Simulator robot with ID " << id << " not found" << std::endl;
    }
}

void ErForceSimulator::stepSimulation(const Duration& time_step)
{
    // Set the ball being referenced in each firmware_world.
    // We only need to do this a single time since all robots
    // can see and interact with the same ball

    current_firmware_time = physics_world.getTimestamp();

    SSLSimulationProto::RobotControl yellow_robot_control;
    SSLSimulationProto::RobotControl blue_robot_control;

    // hack to make things move
    auto move_command = createRobotMoveCommand(
        100, 200, -300, -400, robot_constants.front_wheel_angle_deg,
        robot_constants.back_wheel_angle_deg, wheel_constants.wheel_radius_meters);

    auto robot_command = createRobotCommand(1, std::move(move_command), 0, 0, 0);

    std::vector<std::unique_ptr<SSLSimulationProto::RobotCommand>> robot_commands;
    robot_commands.emplace_back(std::move(robot_command));

    auto yellow_robot_control_ptr = createRobotControl(std::move(robot_commands));

    for (auto& iter : blue_simulator_robots)
    {
        auto simulator_robot = iter.first;
        auto firmware_world  = iter.second;

        app_logger_init(simulator_robot->getRobotId(),
                        &ErForceSimulatorRobotSingleton::handleBlueRobotLogProto);

        std::cout << "stepSimulation: blue_team_vision_msg = "
                  << blue_team_vision_msg->DebugString() << std::endl;
        auto robot_state_it =
            blue_team_vision_msg->robot_states().find(simulator_robot->getRobotId());
        if (robot_state_it == blue_team_vision_msg->robot_states().end())
        {
            std::cout << "Robot state for robot " << simulator_robot->getRobotId()
                      << " not found";
        }
        else
        {
            simulator_robot->setRobotState(RobotState(
                Point(blue_team_vision_msg->robot_states()
                          .at(simulator_robot->getRobotId())
                          .global_position()
                          .x_meters(),
                      blue_team_vision_msg->robot_states()
                          .at(simulator_robot->getRobotId())
                          .global_position()
                          .y_meters()),
                Vector(blue_team_vision_msg->robot_states()
                           .at(simulator_robot->getRobotId())
                           .global_velocity()
                           .x_component_meters(),
                       blue_team_vision_msg->robot_states()
                           .at(simulator_robot->getRobotId())
                           .global_velocity()
                           .x_component_meters()),
                Angle::fromRadians(blue_team_vision_msg->robot_states()
                                       .at(simulator_robot->getRobotId())
                                       .global_orientation()
                                       .radians()),
                // TODO check that blue_team_vision_msg has a msg for robot at that id
                AngularVelocity::fromRadians(blue_team_vision_msg->robot_states()
                                                 .at(simulator_robot->getRobotId())
                                                 .global_angular_velocity()
                                                 .radians_per_second())));
            ErForceSimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
            simulator_ball = std::make_shared<ErForceSimulatorBall>(BallState(
                Point(blue_team_vision_msg->ball_state().global_position().x_meters(),
                      blue_team_vision_msg->ball_state().global_position().y_meters()),
                Vector(blue_team_vision_msg->ball_state()
                           .global_velocity()
                           .x_component_meters(),
                       blue_team_vision_msg->ball_state()
                           .global_velocity()
                           .x_component_meters())));
            SimulatorBallSingleton::setSimulatorBall(simulator_ball, FieldSide::NEG_X);
            ErForceSimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(
                firmware_world);
            *(blue_robot_control.mutable_robot_commands()->Add()) =
                *(simulator_robot->getRobotCommand());
        }
    }

    for (auto& iter : yellow_simulator_robots)
    {
        auto simulator_robot = iter.first;
        auto firmware_world  = iter.second;

        app_logger_init(simulator_robot->getRobotId(),
                        &ErForceSimulatorRobotSingleton::handleYellowRobotLogProto);
        auto robot_state_it =
            yellow_team_vision_msg->robot_states().find(simulator_robot->getRobotId());

        std::cout << "stepSimulation: yellow_team_vision_msg = "
                  << yellow_team_vision_msg->DebugString() << std::endl;
        if (robot_state_it == yellow_team_vision_msg->robot_states().end())
        {
            std::cout << "Robot state for robot " << simulator_robot->getRobotId()
                      << " not found";
        }
        else
        {
            simulator_robot->setRobotState(RobotState(
                Point(yellow_team_vision_msg->robot_states()
                          .at(simulator_robot->getRobotId())
                          .global_position()
                          .x_meters(),
                      yellow_team_vision_msg->robot_states()
                          .at(simulator_robot->getRobotId())
                          .global_position()
                          .y_meters()),
                Vector(yellow_team_vision_msg->robot_states()
                           .at(simulator_robot->getRobotId())
                           .global_velocity()
                           .x_component_meters(),
                       yellow_team_vision_msg->robot_states()
                           .at(simulator_robot->getRobotId())
                           .global_velocity()
                           .x_component_meters()),
                Angle::fromRadians(yellow_team_vision_msg->robot_states()
                                       .at(simulator_robot->getRobotId())
                                       .global_orientation()
                                       .radians()),
                AngularVelocity::fromRadians(yellow_team_vision_msg->robot_states()
                                                 .at(simulator_robot->getRobotId())
                                                 .global_angular_velocity()
                                                 .radians_per_second())));
            ErForceSimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
            simulator_ball = std::make_shared<ErForceSimulatorBall>(BallState(
                Point(yellow_team_vision_msg->ball_state().global_position().x_meters(),
                      yellow_team_vision_msg->ball_state().global_position().y_meters()),
                Vector(yellow_team_vision_msg->ball_state()
                           .global_velocity()
                           .x_component_meters(),
                       yellow_team_vision_msg->ball_state()
                           .global_velocity()
                           .x_component_meters())));
            SimulatorBallSingleton::setSimulatorBall(simulator_ball, FieldSide::NEG_X);
            ErForceSimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(
                firmware_world);

            *(yellow_robot_control.mutable_robot_commands()->Add()) =
                *(simulator_robot->getRobotCommand());
        }
    }

    er_force_sim->acceptBlueRobotControlCommand(blue_robot_control);
    er_force_sim->acceptYellowRobotControlCommand(yellow_robot_control);
    // er_force_sim->acceptYellowRobotControlCommand(*yellow_robot_control_ptr);
    er_force_sim->stepSimulation(time_step.toSeconds());

    frame_number++;
}

std::vector<SSLProto::SSL_WrapperPacket> ErForceSimulator::getSSLWrapperPackets() const
{
    return er_force_sim->getWrapperPackets();
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
