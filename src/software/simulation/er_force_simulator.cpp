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
#include "software/world/robot_state.h"

ErForceSimulator::ErForceSimulator(
    const FieldType& field_type, const RobotConstants_t& robot_constants,
    const WheelConstants& wheel_constants,
    std::shared_ptr<const SimulatorConfig> simulator_config)
    : yellow_team_vision_msg(std::make_unique<TbotsProto::Vision>()),
      blue_team_vision_msg(std::make_unique<TbotsProto::Vision>()),
      frame_number(0),
      robot_constants(robot_constants),
      wheel_constants(wheel_constants),
      field(Field::createField(field_type))
{
    QString full_filename = CONFIG_DIRECTORY;

    if (field_type == FieldType::DIV_A)
    {
        full_filename = full_filename + CONFIG_FILE + ".txt";
    }
    else
    {
        // loading division B configuration
        full_filename = full_filename + CONFIG_FILE + "B.txt";
    }

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
    er_force_sim = std::make_unique<camun::simulator::Simulator>(er_force_sim_setup);
    auto simulator_setup_command = std::make_unique<amun::Command>();
    simulator_setup_command->mutable_simulator()->set_enable(true);
    // start with default robots, take ER-Force specs.
    robot::Specs ERForce;
    robotSetDefault(&ERForce);
    Team friendly_team = Team();
    Team enemy_team    = Team();
    Ball ball          = Ball(Point(), Vector(), Timestamp::fromSeconds(0));
    World world        = World(field, ball, friendly_team, enemy_team);

    auto realism_config = std::make_unique<RealismConfigErForce>();
    // Sets the dribbler to be ideal
    realism_config->set_simulate_dribbling(false);
    auto command_simulator = std::make_unique<amun::CommandSimulator>();
    *(command_simulator->mutable_realism_config())  = *realism_config;
    *(simulator_setup_command->mutable_simulator()) = *command_simulator;

    er_force_sim->handleSimulatorSetupCommand(simulator_setup_command);

    this->resetCurrentTime();
}

void ErForceSimulator::setBallState(const BallState& ball_state)
{
    auto simulator_setup_command = std::make_unique<amun::Command>();
    auto teleport_ball           = std::make_unique<sslsim::TeleportBall>();
    auto simulator_control       = std::make_unique<sslsim::SimulatorControl>();
    auto command_simulator       = std::make_unique<amun::CommandSimulator>();

    teleport_ball->set_x(
        static_cast<float>(ball_state.position().x() * MILLIMETERS_PER_METER));
    teleport_ball->set_y(
        static_cast<float>(ball_state.position().y() * MILLIMETERS_PER_METER));
    teleport_ball->set_vx(
        static_cast<float>(ball_state.velocity().x() * MILLIMETERS_PER_METER));
    teleport_ball->set_vy(
        static_cast<float>(ball_state.velocity().y() * MILLIMETERS_PER_METER));
    *(simulator_control->mutable_teleport_ball())   = *teleport_ball;
    *(command_simulator->mutable_ssl_control())     = *simulator_control;
    *(simulator_setup_command->mutable_simulator()) = *command_simulator;

    er_force_sim->handleSimulatorSetupCommand(simulator_setup_command);
}

void ErForceSimulator::setYellowRobots(const std::vector<RobotStateWithId>& robots)
{
    setRobots(robots, gameController::Team::YELLOW);
}

void ErForceSimulator::setBlueRobots(const std::vector<RobotStateWithId>& robots)
{
    setRobots(robots, gameController::Team::BLUE);
}

void ErForceSimulator::setRobots(const std::vector<RobotStateWithId>& robots,
                                 gameController::Team side)
{
    auto simulator_setup_command = std::make_unique<amun::Command>();

    robot::Specs ERForce;
    robotSetDefault(&ERForce);

    // Initialize Team Robots at the bottom of the field
    ::robot::Team* team;
    if (side == gameController::Team::BLUE)
    {
        team = simulator_setup_command->mutable_set_team_blue();
    }
    else
    {
        team = simulator_setup_command->mutable_set_team_yellow();
    }

    for (const auto& robot_state_with_id : robots)
    {
        auto* robot = team->add_robot();
        robot->CopyFrom(ERForce);
        robot->set_id(robot_state_with_id.id);
    }
    er_force_sim->handleSimulatorSetupCommand(simulator_setup_command);

    if (side == gameController::Team::BLUE)
    {
        simulator_setup_command->clear_set_team_blue();
    }
    else
    {
        simulator_setup_command->clear_set_team_yellow();
    }

    auto simulator_control = std::make_shared<sslsim::SimulatorControl>();
    auto command_simulator = std::make_unique<amun::CommandSimulator>();

    // Add each robot to be added to the teleport robot repeated field
    for (const auto& robot_state_with_id : robots)
    {
        auto teleport_robot             = std::make_unique<sslsim::TeleportRobot>();
        gameController::BotId* robot_id = new gameController::BotId();
        robot_id->set_id(robot_state_with_id.id);

        if (side == gameController::Team::BLUE)
        {
            robot_id->set_team(gameController::Team::BLUE);
        }
        else
        {
            robot_id->set_team(gameController::Team::YELLOW);
        }

        teleport_robot->set_x(static_cast<float>(
            robot_state_with_id.robot_state.position().x() * MILLIMETERS_PER_METER));
        teleport_robot->set_y(static_cast<float>(
            robot_state_with_id.robot_state.position().y() * MILLIMETERS_PER_METER));
        teleport_robot->set_allocated_id(robot_id);
        teleport_robot->set_present(true);

        teleport_robot->set_orientation(static_cast<float>(
            robot_state_with_id.robot_state.orientation().toRadians()));

        teleport_robot->set_v_x(static_cast<float>(
            robot_state_with_id.robot_state.velocity().x() * MILLIMETERS_PER_METER));
        teleport_robot->set_v_y(static_cast<float>(
            robot_state_with_id.robot_state.velocity().y() * MILLIMETERS_PER_METER));
        teleport_robot->set_v_angular(static_cast<float>(
            robot_state_with_id.robot_state.angularVelocity().toRadians()));

        *(simulator_control->add_teleport_robot()) = *teleport_robot;
    }

    // Send message to simulator to teleport robots
    *(command_simulator->mutable_ssl_control())     = *simulator_control;
    *(simulator_setup_command->mutable_simulator()) = *command_simulator;
    er_force_sim->handleSimulatorSetupCommand(simulator_setup_command);

    if (side == gameController::Team::BLUE)
    {
        blue_simulator_robots.clear();
    }
    else
    {
        yellow_simulator_robots.clear();
    }

    for (const auto& robot_state_with_id : robots)
    {
        auto simulator_robot = std::make_shared<ErForceSimulatorRobot>(
            RobotStateWithId{.id          = robot_state_with_id.id,
                             .robot_state = robot_state_with_id.robot_state},
            robot_constants, wheel_constants);

        if (side == gameController::Team::BLUE)
        {
            blue_simulator_robots.emplace_back(simulator_robot);
        }
        else
        {
            yellow_simulator_robots.emplace_back(simulator_robot);
        }
    }
}

void ErForceSimulator::setYellowRobotPrimitiveSet(
    const TbotsProto::PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::Vision> vision_msg)
{
    for (auto& [robot_id, primitive] : primitive_set_msg.robot_primitives())
    {
        setRobotPrimitive(robot_id, primitive, yellow_simulator_robots,
                          *yellow_team_vision_msg);
    }
    yellow_team_vision_msg = std::move(vision_msg);
}

void ErForceSimulator::setBlueRobotPrimitiveSet(
    const TbotsProto::PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::Vision> vision_msg)
{
    for (auto& [robot_id, primitive] : primitive_set_msg.robot_primitives())
    {
        setRobotPrimitive(robot_id, primitive, blue_simulator_robots,
                          *blue_team_vision_msg);
    }
    blue_team_vision_msg = std::move(vision_msg);
}

void ErForceSimulator::setRobotPrimitive(
    RobotId id, const TbotsProto::Primitive& primitive_msg,
    std::vector<std::shared_ptr<ErForceSimulatorRobot>>& simulator_robots,
    const TbotsProto::Vision& vision_msg)
{
    // Set to NEG_X because the vision msg in this simulator is normalized
    // correctly
    auto simulator_robots_iter =
        std::find_if(simulator_robots.begin(), simulator_robots.end(),
                     [id](const auto& robot) { return robot->getRobotId() == id; });

    if (simulator_robots_iter != simulator_robots.end())
    {
        auto simulator_robot = *simulator_robots_iter;

        auto robot_state_it = vision_msg.robot_states().find(id);
        if (robot_state_it != vision_msg.robot_states().end())
        {
            simulator_robot->setRobotState(RobotState(vision_msg.robot_states().at(id)));
            simulator_robot->startNewPrimitive(primitive_msg);
        }
    }
    else
    {
        LOG(WARNING) << "Simulator robot with ID " << id << " not found" << std::endl;
    }
}

SSLSimulationProto::RobotControl ErForceSimulator::updateSimulatorRobots(
    std::vector<std::shared_ptr<ErForceSimulatorRobot>> simulator_robots,
    TbotsProto::Vision vision_msg)
{
    SSLSimulationProto::RobotControl robot_control;

    for (auto& simulator_robot : simulator_robots)
    {
        auto robot_state_it =
            vision_msg.robot_states().find(simulator_robot->getRobotId());
        if (robot_state_it != vision_msg.robot_states().end())
        {
            simulator_robot->setRobotState(
                RobotState(vision_msg.robot_states().at(simulator_robot->getRobotId())));
            // Set to NEG_X because the vision msg in this simulator is
            // normalized correctly
            simulator_robot->runCurrentPrimitive();
            auto command = *simulator_robot->getRobotCommand();
            *(robot_control.mutable_robot_commands()->Add()) = command;
        }
    }
    return robot_control;
}

void ErForceSimulator::stepSimulation(const Duration& time_step)
{
    current_time = current_time + time_step;

    SSLSimulationProto::RobotControl yellow_robot_control =
        updateSimulatorRobots(yellow_simulator_robots, *yellow_team_vision_msg);

    SSLSimulationProto::RobotControl blue_robot_control =
        updateSimulatorRobots(blue_simulator_robots, *blue_team_vision_msg);

    er_force_sim->acceptYellowRobotControlCommand(yellow_robot_control);
    er_force_sim->acceptBlueRobotControlCommand(blue_robot_control);
    er_force_sim->stepSimulation(time_step.toSeconds());

    frame_number++;
}

std::vector<SSLProto::SSL_WrapperPacket> ErForceSimulator::getSSLWrapperPackets() const
{
    return er_force_sim->getWrapperPackets();
}

world::SimulatorState ErForceSimulator::getSimulatorState() const
{
    return er_force_sim->getSimulatorState();
}

Field ErForceSimulator::getField() const
{
    return field;
}

Timestamp ErForceSimulator::getTimestamp() const
{
    return current_time;
}

void ErForceSimulator::resetCurrentTime()
{
    current_time = Timestamp::fromSeconds(0);
}
