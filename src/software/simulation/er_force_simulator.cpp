#include "software/simulation/er_force_simulator.h"

#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>

#include <QtCore/QFile>
#include <QtCore/QString>
#include <iostream>

#include "extlibs/er_force_sim/src/protobuf/robot.h"
#include "proto/message_translation/ssl_detection.h"
#include "proto/message_translation/ssl_geometry.h"
#include "proto/message_translation/ssl_simulation_robot_control.h"
#include "proto/message_translation/ssl_wrapper.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/robot_status_msg.pb.h"
#include "software/logger/logger.h"
#include "software/world/robot_state.h"

ErForceSimulator::ErForceSimulator(const TbotsProto::FieldType& field_type,
                                   const RobotConstants_t& robot_constants,
                                   const WheelConstants& wheel_constants)
    : yellow_team_world_msg(std::make_unique<TbotsProto::World>()),
      blue_team_world_msg(std::make_unique<TbotsProto::World>()),
      frame_number(0),
      robot_constants(robot_constants),
      wheel_constants(wheel_constants),
      field(Field::createField(field_type)),
      blue_robot_with_ball(std::nullopt),
      yellow_robot_with_ball(std::nullopt)
{
    QString full_filename = CONFIG_DIRECTORY;

    if (field_type == TbotsProto::FieldType::DIV_A)
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

void ErForceSimulator::setWorldState(const TbotsProto::WorldState& world_state)
{
    if (world_state.has_ball_state())
    {
        setBallState(createBallState(world_state.ball_state()));
    }
    if (world_state.blue_robots().size() > 0)
    {
        setRobots(world_state.blue_robots(), gameController::Team::BLUE);
    }
    if (world_state.yellow_robots().size() > 0)
    {
        setRobots(world_state.yellow_robots(), gameController::Team::YELLOW);
    }
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
    google::protobuf::Map<uint32_t, TbotsProto::RobotState> proto_robots;
    for (const auto& robot_state_with_id : robots)
    {
        proto_robots[robot_state_with_id.id] =
            *createRobotStateProto(robot_state_with_id.robot_state);
    }
    setRobots(proto_robots, side);
}

void ErForceSimulator::setRobots(
    const google::protobuf::Map<uint32_t, TbotsProto::RobotState>& robots,
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

    for (auto& [id, robot_state] : robots)
    {
        auto* robot = team->add_robot();
        robot->CopyFrom(ERForce);
        robot->set_id(id);
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
    for (auto& [id, robot_state] : robots)
    {
        auto teleport_robot             = std::make_unique<sslsim::TeleportRobot>();
        gameController::BotId* robot_id = new gameController::BotId();
        robot_id->set_id(static_cast<int>(id));

        if (side == gameController::Team::BLUE)
        {
            robot_id->set_team(gameController::Team::BLUE);
        }
        else
        {
            robot_id->set_team(gameController::Team::YELLOW);
        }

        teleport_robot->set_x(static_cast<float>(
            robot_state.global_position().x_meters() * MILLIMETERS_PER_METER));
        teleport_robot->set_y(static_cast<float>(
            robot_state.global_position().y_meters() * MILLIMETERS_PER_METER));
        teleport_robot->set_allocated_id(robot_id);
        teleport_robot->set_present(true);

        teleport_robot->set_orientation(
            static_cast<float>(robot_state.global_orientation().radians()));

        teleport_robot->set_v_x(static_cast<float>(
            robot_state.global_velocity().x_component_meters() * MILLIMETERS_PER_METER));
        teleport_robot->set_v_y(static_cast<float>(
            robot_state.global_velocity().y_component_meters() * MILLIMETERS_PER_METER));
        teleport_robot->set_v_angular(static_cast<float>(
            robot_state.global_angular_velocity().radians_per_second()));

        *(simulator_control->add_teleport_robot()) = *teleport_robot;
    }

    // Send message to simulator to teleport robots
    *(command_simulator->mutable_ssl_control())     = *simulator_control;
    *(simulator_setup_command->mutable_simulator()) = *command_simulator;
    er_force_sim->handleSimulatorSetupCommand(simulator_setup_command);

    if (side == gameController::Team::BLUE)
    {
        blue_primitive_executor_map.clear();
    }
    else
    {
        yellow_primitive_executor_map.clear();
    }

    for (auto& [id, robot_state] : robots)
    {
        if (side == gameController::Team::BLUE)
        {
            blue_primitive_executor_map.insert({id, robot_primitive_executor});
        }
        else
        {
            yellow_primitive_executor_map.insert({id, robot_primitive_executor});
        }
    }
}

void ErForceSimulator::setYellowRobotPrimitiveSet(
    const TbotsProto::PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::World> world_msg)
{
    auto sim_state  = getSimulatorState();
    auto sim_robots = sim_state.yellow_robots();
    std::map<RobotId, Vector> robot_to_local_velocity =
        getRobotIdToLocalVelocityMap(sim_robots);

    for (auto& [robot_id, primitive] : primitive_set_msg.robot_primitives())
    {
        setRobotPrimitive(robot_id, primitive_set_msg, yellow_primitive_executor_map,
                          *yellow_team_world_msg, robot_to_local_velocity.at(robot_id));
    }
    yellow_team_world_msg = std::move(world_msg);
}

void ErForceSimulator::setBlueRobotPrimitiveSet(
    const TbotsProto::PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::World> world_msg)
{
    auto sim_state  = getSimulatorState();
    auto sim_robots = sim_state.blue_robots();
    std::map<RobotId, Vector> robot_to_local_velocity =
        getRobotIdToLocalVelocityMap(sim_robots);

    for (auto& [robot_id, primitive] : primitive_set_msg.robot_primitives())
    {
        setRobotPrimitive(robot_id, primitive_set_msg, blue_primitive_executor_map,
                          *blue_team_world_msg, robot_to_local_velocity.at(robot_id));
    }
    blue_team_world_msg = std::move(world_msg);
}

void ErForceSimulator::setRobotPrimitive(
    RobotId id, const TbotsProto::PrimitiveSet& primitive_set_msg,
    std::unordered_map<unsigned int, std::shared_ptr<PrimitiveExecutor>>&
        robot_primitive_executor_map,
    const TbotsProto::World& world_msg, Vector local_velocity)
{
    // Set to NEG_X because the world msg in this simulator is normalized
    // correctly
    auto robot_primitive_executor_iter = robot_primitive_executor_map.find(id);

    if (robot_primitive_executor_iter != robot_primitive_executor_map.end())
    {
        auto robot_primitive_executor = robot_primitive_executor_iter->second;
        unsigned int robot_id         = robot_primitive_executor_iter->first;

        const auto& friendly_robots = world_msg.friendly_team().team_robots();
        const auto robot_proto_it =
            std::find_if(friendly_robots.begin(), friendly_robots.end(),
                         [&](const auto& robot) { return robot.id() == robot_id; });
        if (robot_proto_it != friendly_robots.end())
        {
            robot_primitive_executor->updatePrimitiveSet(robot_id, primitive_set_msg);
            robot_primitive_executor->updateWorld(world_msg);
            robot_primitive_executor->updateLocalVelocity(local_velocity);
        }
        else
        {
            LOG(WARNING) << "Robot with ID " << id
                         << " not included in world proto message" << std::endl;
        }
    }
    else
    {
        LOG(WARNING) << "Primitive Executor for robot with ID " << id << " not found"
                     << std::endl;
    }
}

SSLSimulationProto::RobotControl ErForceSimulator::updateSimulatorRobots(
    std::unordered_map<unsigned int, std::shared_ptr<PrimitiveExecutor>>&
        robot_primitive_executor_map,
    const TbotsProto::World& world_msg)
{
    SSLSimulationProto::RobotControl robot_control;

    for (auto& primitive_executor_with_id : robot_primitive_executor_map)
    {
        unsigned int robot_id       = primitive_executor_with_id.first;
        const auto& friendly_robots = world_msg.friendly_team().team_robots();
        const auto& robot_proto_it =
            std::find_if(friendly_robots.begin(), friendly_robots.end(),
                         [&](const auto& robot) { return robot.id() == robot_id; });
        if (robot_proto_it != friendly_robots.end())
        {
            auto& primitive_executor = primitive_executor_with_id.second;
            // Set to NEG_X because the world msg in this simulator is
            // normalized correctly
            auto direct_control = primitive_executor->stepPrimitive(
                robot_id, RobotState(robot_proto_it->current_state()).orientation());

            auto command = *getRobotCommandFromDirectControl(
                robot_id, std::move(direct_control), robot_constants, wheel_constants);
            *(robot_control.mutable_robot_commands()->Add()) = command;
        }
    }
    return robot_control;
}

void ErForceSimulator::stepSimulation(const Duration& time_step)
{
    current_time = current_time + time_step;

    SSLSimulationProto::RobotControl yellow_robot_control =
        updateSimulatorRobots(yellow_primitive_executor_map, *yellow_team_world_msg);

    SSLSimulationProto::RobotControl blue_robot_control =
        updateSimulatorRobots(blue_primitive_executor_map, *blue_team_world_msg);

    auto yellow_radio_responses =
        er_force_sim->acceptYellowRobotControlCommand(yellow_robot_control);
    auto blue_radio_responses =
        er_force_sim->acceptBlueRobotControlCommand(blue_robot_control);

    blue_robot_with_ball.reset();
    yellow_robot_with_ball.reset();

    for (const auto& response : yellow_radio_responses)
    {
        if (response.has_ball_detected() && response.ball_detected())
        {
            yellow_robot_with_ball = response.id();
        }
    }

    for (const auto& response : blue_radio_responses)
    {
        if (response.has_ball_detected() && response.ball_detected())
        {
            blue_robot_with_ball = response.id();
        }
    }

    er_force_sim->stepSimulation(time_step.toSeconds());

    frame_number++;
}

std::vector<TbotsProto::RobotStatus> ErForceSimulator::getBlueRobotStatuses() const
{
    std::vector<TbotsProto::RobotStatus> robot_statuses;
    auto robot_status      = TbotsProto::RobotStatus();
    auto break_beam_status = TbotsProto::BreakBeamStatus();

    if (blue_robot_with_ball.has_value())
    {
        robot_status.set_robot_id(blue_robot_with_ball.value());
        break_beam_status.set_ball_in_beam(true);
    }
    else
    {
        robot_status.clear_robot_id();
        break_beam_status.set_ball_in_beam(false);
    }

    *(robot_status.mutable_break_beam_status()) = break_beam_status;
    robot_statuses.push_back(robot_status);

    return robot_statuses;
}

std::vector<TbotsProto::RobotStatus> ErForceSimulator::getYellowRobotStatuses() const
{
    std::vector<TbotsProto::RobotStatus> robot_statuses;
    auto robot_status      = TbotsProto::RobotStatus();
    auto break_beam_status = TbotsProto::BreakBeamStatus();

    if (yellow_robot_with_ball.has_value())
    {
        robot_status.set_robot_id(yellow_robot_with_ball.value());
        break_beam_status.set_ball_in_beam(true);
    }
    else
    {
        robot_status.clear_robot_id();
        break_beam_status.set_ball_in_beam(false);
    }

    *(robot_status.mutable_break_beam_status()) = break_beam_status;
    robot_statuses.push_back(robot_status);

    return robot_statuses;
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

std::map<RobotId, Vector> ErForceSimulator::getRobotIdToLocalVelocityMap(
    const google::protobuf::RepeatedPtrField<world::SimRobot>& sim_robots)
{
    std::map<RobotId, Vector> robot_to_local_velocity;
    for (const auto& sim_robot : sim_robots)
    {
        // rotate converts global velocity to local velocity
        robot_to_local_velocity[sim_robot.id()] =
            Vector(sim_robot.v_x(), sim_robot.v_y())
                .rotate(Angle::fromRadians(sim_robot.angle()));
    }
    return robot_to_local_velocity;
}
