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
#include "proto/message_translation/tbots_protobuf.h"
#include "software/simulation/er_force_simulator_robot_singleton.h"
#include "software/simulation/simulator_ball_singleton.h"
#include "software/world/robot_state.h"

extern "C"
{
#include "firmware/app/logger/logger.h"
#include "firmware/app/world/firmware_ball.h"
#include "firmware/app/world/firmware_robot.h"
#include "firmware/app/world/firmware_world.h"
}

ErForceSimulator::ErForceSimulator(
    const Field& field, const RobotConstants_t& robot_constants,
    const WheelConstants& wheel_constants,
    std::shared_ptr<const SimulatorConfig> simulator_config)
    : yellow_team_vision_msg(std::make_unique<TbotsProto::Vision>()),
      blue_team_vision_msg(std::make_unique<TbotsProto::Vision>()),
      frame_number(0),
      robot_constants(robot_constants),
      wheel_constants(wheel_constants)
{
    QString config_file("simulator/2020");
    QString full_filename =
        QString("extlibs/er_force_sim/config/") + config_file + ".txt";
    QFile file(full_filename);
    if (!file.open(QFile::ReadOnly))
    {
        LOG(FATAL) << "Could not open configuration file " << full_filename.toStdString()
                   << std::endl;
    }
    QString str = file.readAll();
    file.close();
    std::string s = qPrintable(str);

    google::protobuf::TextFormat::Parser parser;
    parser.ParseFromString(s, &er_force_sim_setup);

    er_force_sim =
        std::make_unique<camun::simulator::Simulator>(er_force_sim_setup, true);

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
    er_force_sim->safelyTeleportBall(static_cast<float>(ball_state.position().x()),
                                     static_cast<float>(ball_state.position().y()));
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
                      *yellow_team_vision_msg);
}

void ErForceSimulator::setBlueRobotPrimitive(RobotId id,
                                             const TbotsProto_Primitive& primitive_msg)
{
    setRobotPrimitive(id, primitive_msg, blue_simulator_robots, simulator_ball,
                      *blue_team_vision_msg);
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
    simulator_ball =
        std::make_shared<ErForceSimulatorBall>(createBallState(vision_msg.ball_state()));
    // Set to NEG_X because the vision msg in this simulator is normalized
    // correctly
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
        if (robot_state_it != vision_msg.robot_states().end())
        {
            simulator_robot->setRobotState(
                createRobotState(vision_msg.robot_states().at(id)));
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

SSLSimulationProto::RobotControl ErForceSimulator::updateSimulatorRobots(
    void (*handle_robot_log_proto)(TbotsProto_RobotLog),
    std::map<std::shared_ptr<ErForceSimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>
        simulator_robots,
    TbotsProto::Vision vision_msg)
{
    SSLSimulationProto::RobotControl robot_control;

    for (auto& iter : simulator_robots)
    {
        auto simulator_robot = iter.first;
        auto firmware_world  = iter.second;

        app_logger_init(simulator_robot->getRobotId(), handle_robot_log_proto);

        auto robot_state_it =
            vision_msg.robot_states().find(simulator_robot->getRobotId());
        if (robot_state_it != vision_msg.robot_states().end())
        {
            simulator_robot->setRobotState(createRobotState(
                vision_msg.robot_states().at(simulator_robot->getRobotId())));
            ErForceSimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
            auto simulator_ball = std::make_shared<ErForceSimulatorBall>(
                createBallState(vision_msg.ball_state()));
            // Set to NEG_X because the vision msg in this simulator is
            // normalized correctly
            SimulatorBallSingleton::setSimulatorBall(simulator_ball, FieldSide::NEG_X);
            ErForceSimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(
                firmware_world);
            *(robot_control.mutable_robot_commands()->Add()) =
                *(simulator_robot->getRobotCommand());
        }
    }
    return robot_control;
}

void ErForceSimulator::stepSimulation(const Duration& time_step)
{
    // Set the ball being referenced in each firmware_world.
    // We only need to do this a single time since all robots
    // can see and interact with the same ball

    current_firmware_time = current_firmware_time + time_step;

    SSLSimulationProto::RobotControl yellow_robot_control =
        updateSimulatorRobots(&ErForceSimulatorRobotSingleton::handleYellowRobotLogProto,
                              yellow_simulator_robots, *yellow_team_vision_msg);

    SSLSimulationProto::RobotControl blue_robot_control =
        updateSimulatorRobots(&ErForceSimulatorRobotSingleton::handleBlueRobotLogProto,
                              blue_simulator_robots, *blue_team_vision_msg);

    er_force_sim->acceptYellowRobotControlCommand(yellow_robot_control);
    er_force_sim->acceptBlueRobotControlCommand(blue_robot_control);
    er_force_sim->stepSimulation(time_step.toSeconds());

    frame_number++;
}

std::vector<SSLProto::SSL_WrapperPacket> ErForceSimulator::getSSLWrapperPackets() const
{
    return er_force_sim->getWrapperPackets();
}

Field ErForceSimulator::getField() const
{
    return Field::createSSLDivisionAField();
}

Timestamp ErForceSimulator::getTimestamp() const
{
    return current_firmware_time;
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
