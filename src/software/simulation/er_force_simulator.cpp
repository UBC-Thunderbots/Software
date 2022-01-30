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
    : yellow_team_world_msg(std::make_unique<TbotsProto::World>()),
      blue_team_world_msg(std::make_unique<TbotsProto::World>()),
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
    auto simulator_setup_command = std::make_shared<amun::Command>();
    simulator_setup_command->mutable_simulator()->set_enable(true);
    // start with default robots, take ER-Force specs.
    robot::Specs ERForce;
    robotSetDefault(&ERForce);
    Team friendly_team = Team();
    Team enemy_team    = Team();
    Ball ball          = Ball(Point(), Vector(), Timestamp::fromSeconds(0));
    World world        = World(field, ball, friendly_team, enemy_team);

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
    std::cout << __func__ << " called" << std::endl;
    setRobots(robots, gameController::Team::YELLOW);
    std::cout << __func__ << " finished" << std::endl;
}

void ErForceSimulator::setBlueRobots(const std::vector<RobotStateWithId>& robots)
{
    std::cout << __func__ << " called" << std::endl;
    setRobots(robots, gameController::Team::BLUE);
    std::cout << __func__ << " finished" << std::endl;
}

void ErForceSimulator::setRobots(const std::vector<RobotStateWithId>& robots,
                                 gameController::Team side)
{
    auto simulator_setup_command = std::make_unique<amun::Command>();
    simulator_setup_command->mutable_simulator()->set_enable(true);

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
        std::unique_ptr<TbotsProto::World> world_msg)
{
    for (auto& [robot_id, primitive] : primitive_set_msg.robot_primitives())
    {
        setRobotPrimitive(robot_id, primitive_set_msg, yellow_simulator_robots,
                          *yellow_team_world_msg);
    }
    yellow_team_world_msg = std::move(world_msg);
}

void ErForceSimulator::setBlueRobotPrimitiveSet(
        const TbotsProto::PrimitiveSet& primitive_set_msg,
        std::unique_ptr<TbotsProto::World> world_msg)
{
    for (auto& [robot_id, primitive] : primitive_set_msg.robot_primitives())
    {
        setRobotPrimitive(robot_id, primitive_set_msg, blue_simulator_robots,
                          *blue_team_world_msg);
    }
    blue_team_world_msg = std::move(world_msg);
}

void ErForceSimulator::setRobotPrimitive(
        RobotId id, const TbotsProto::PrimitiveSet &primitive_set_msg,
        std::vector<std::shared_ptr<ErForceSimulatorRobot>>& simulator_robots,
        const TbotsProto::World &world_msg)
{
    // Set to NEG_X because the vision msg in this simulator is normalized
    // correctly
    auto simulator_robots_iter =
        std::find_if(simulator_robots.begin(), simulator_robots.end(),
                     [id](const auto& robot) { return robot->getRobotId() == id; });

    if (simulator_robots_iter != simulator_robots.end())
    {
        auto simulator_robot = *simulator_robots_iter;

        const auto &friendly_robots = world_msg.friendly_team().team_robots();
        const auto robot_proto_it = std::find_if(friendly_robots.begin(), friendly_robots.end(), [&](const auto& robot) {
            return robot.id() == simulator_robot->getRobotId();
        });
        if (robot_proto_it != friendly_robots.end())
        {
            simulator_robot->setRobotState(RobotState(robot_proto_it->current_state()));
            simulator_robot->startNewPrimitiveSet(primitive_set_msg);
            simulator_robot->updateWorldState(world_msg);
        }
    }
    else
    {
        LOG(WARNING) << "Simulator robot with ID " << id << " not found" << std::endl;
    }
}

SSLSimulationProto::RobotControl ErForceSimulator::updateSimulatorRobots(
        const std::vector<std::shared_ptr<ErForceSimulatorRobot>>& simulator_robots,
        const TbotsProto::World& world_msg)
{
    std::cout << __func__ << " start" << std::endl;
    SSLSimulationProto::RobotControl robot_control;

    for (auto& simulator_robot : simulator_robots)
    {
        const auto &friendly_robots = world_msg.friendly_team().team_robots();
        const auto& robot_proto_it = std::find_if(friendly_robots.begin(), friendly_robots.end(), [&](const auto& robot) {
            return robot.id() == simulator_robot->getRobotId();
        });
        if (robot_proto_it != friendly_robots.end())
        {
            simulator_robot->setRobotState(RobotState(robot_proto_it->current_state()));
            // Set to NEG_X because the vision msg in this simulator is
            // normalized correctly
            simulator_robot->runCurrentPrimitive();
            auto command = *simulator_robot->getRobotCommand();
            *(robot_control.mutable_robot_commands()->Add()) = command;
        }
    }
    std::cout << __func__ << " end" << std::endl;
    return robot_control;
}

void ErForceSimulator::stepSimulation(const Duration& time_step)
{
    std::cout << __func__ << " start" << std::endl;
    current_time = current_time + time_step;

    SSLSimulationProto::RobotControl yellow_robot_control =
        updateSimulatorRobots(yellow_simulator_robots, *yellow_team_world_msg);

    SSLSimulationProto::RobotControl blue_robot_control =
        updateSimulatorRobots(blue_simulator_robots, *blue_team_world_msg);

    er_force_sim->acceptYellowRobotControlCommand(yellow_robot_control);
    er_force_sim->acceptBlueRobotControlCommand(blue_robot_control);
    er_force_sim->stepSimulation(time_step.toSeconds());

    frame_number++;
    std::cout << __func__ << " end" << std::endl;
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
