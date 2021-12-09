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
#include "extlibs/er_force_sim/src/amun/simulator/simulator.h"

extern "C"
{
#include "firmware/app/logger/logger.h"
#include "firmware/app/world/firmware_ball.h"
#include "firmware/app/world/firmware_robot.h"
#include "firmware/app/world/firmware_world.h"
}

MAKE_ENUM(FieldType, DIV_A, DIV_B);

ErForceSimulator::ErForceSimulator(
    const FieldType& field_type, const RobotConstants_t& robot_constants,
    const WheelConstants& wheel_constants, 
    std::shared_ptr<const SimulatorConfig> simulator_config)
    : yellow_team_vision_msg(std::make_unique<TbotsProto::Vision>()),
      blue_team_vision_msg(std::make_unique<TbotsProto::Vision>()),
      frame_number(0),
      robot_constants(robot_constants),
      wheel_constants(wheel_constants)
{
    QString full_filename = CONFIG_DIRECTORY + CONFIG_FILE + ".txt";
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

    auto simulator_setup_command = std::make_shared<amun::Command>();
    simulator_setup_command->mutable_simulator()->set_enable(true);
    // start with default robots, take ER-Force specs.
    robot::Specs ERForce;
    robotSetDefault(&ERForce);

    Team friendly_team = Team();
    Team enemy_team    = Team();
    Ball ball          = Ball(Point(), Vector(), Timestamp::fromSeconds(0));

    World world = World(field, ball, friendly_team, enemy_team);

    // TODO (#2283): remove this initialization when addYellowRobots and addBlueRobots are
    // implemented
    auto* team_blue   = simulator_setup_command->mutable_set_team_blue();
    auto* team_yellow = simulator_setup_command->mutable_set_team_yellow();
    for (auto* team : {team_blue, team_yellow})
    {
        for (int i = 0; i < 11; ++i)
        {
            auto* robot = team->add_robot();
            robot->CopyFrom(ERForce);
            robot->set_id(i);
        }
    }
    er_force_sim->handleSimulatorSetupCommand(simulator_setup_command);
    er_force_sim->seedPRGN(17);

    // TODO (#2283): remove this initialization when addYellowRobots and addBlueRobots are
    // implemented
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
            yellow_firmware_robot.release(), yellow_firmware_ball.release(),
            &(ErForceSimulator::getCurrentFirmwareTimeSeconds));
        auto yellow_firmware_world = std::shared_ptr<FirmwareWorld_t>(
            yellow_firmware_world_raw, FirmwareWorldDeleter());

        blue_simulator_robots.insert(
            std::make_pair(blue_simulator_robot, blue_firmware_world));
        yellow_simulator_robots.insert(
            std::make_pair(yellow_simulator_robot, yellow_firmware_world));
    }


    this->resetCurrentFirmwareTime();

    field_type = this->getSimulatorSetup();
}

void ErForceSimulator::setBallState(const BallState& ball_state)
{
    er_force_sim->safelyTeleportBall(static_cast<float>(ball_state.position().x()),
                                     static_cast<float>(ball_state.position().y()));
}

void ErForceSimulator::addYellowRobots(const std::vector<RobotStateWithId>& robots)
{
    // TODO (#2283): add robots
}

void ErForceSimulator::addBlueRobots(const std::vector<RobotStateWithId>& robots)
{
    // TODO (#2283): add robots
}

void ErForceSimulator::setYellowRobotPrimitiveSet(
    const TbotsProto_PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::Vision> vision_msg)
{
    for (pb_size_t i = 0; i < primitive_set_msg.robot_primitives_count; i++)
    {
        setRobotPrimitive(primitive_set_msg.robot_primitives[i].key,
                          primitive_set_msg.robot_primitives[i].value,
                          yellow_simulator_robots, simulator_ball,
                          *yellow_team_vision_msg);
    }
    yellow_team_vision_msg = std::move(vision_msg);
}

void ErForceSimulator::setBlueRobotPrimitiveSet(
    const TbotsProto_PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::Vision> vision_msg)
{
    for (pb_size_t i = 0; i < primitive_set_msg.robot_primitives_count; i++)
    {
        setRobotPrimitive(primitive_set_msg.robot_primitives[i].key,
                          primitive_set_msg.robot_primitives[i].value,
                          blue_simulator_robots, simulator_ball, *blue_team_vision_msg);
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

std::unique_ptr<amun::SimulatorSetup> ErForceSimulator::getSimulatorSetup()
{
    return amun::SimulatorSetup();
}


// We must give this variable a value here, as non-const static variables must be
// initialized out-of-line
Timestamp ErForceSimulator::current_firmware_time = Timestamp::fromSeconds(0);
