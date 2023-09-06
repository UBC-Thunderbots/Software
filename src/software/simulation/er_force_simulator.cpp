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
#include "software/physics/velocity_conversion_util.h"
#include "software/world/robot_state.h"

#include "external/tracy/public/tracy/Tracy.hpp"

ErForceSimulator::ErForceSimulator(const TbotsProto::FieldType& field_type,
                                   const RobotConstants_t& robot_constants,
                                   std::unique_ptr<RealismConfigErForce>& realism_config,
                                   const bool ramping)
    : yellow_team_world_msg(std::make_unique<TbotsProto::World>()),
      blue_team_world_msg(std::make_unique<TbotsProto::World>()),
      frame_number(0),
      euclidean_to_four_wheel(robot_constants),
      robot_constants(robot_constants),
      field(Field::createField(field_type)),
      blue_robot_with_ball(std::nullopt),
      yellow_robot_with_ball(std::nullopt),
      ramping(ramping)
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

    /* configure simulator */
    auto command_simulator = std::make_unique<amun::CommandSimulator>();
    *(command_simulator->mutable_realism_config())  = *realism_config;
    *(simulator_setup_command->mutable_simulator()) = *command_simulator;

    er_force_sim->handleSimulatorSetupCommand(simulator_setup_command);

    this->resetCurrentTime();
}

std::unique_ptr<RealismConfigErForce> ErForceSimulator::createDefaultRealismConfig()
{
    auto realism_config = std::make_unique<RealismConfigErForce>();
    realism_config->set_stddev_ball_p(0);
    realism_config->set_stddev_robot_p(0);
    realism_config->set_stddev_robot_phi(0);
    realism_config->set_stddev_ball_area(0);
    realism_config->set_enable_invisible_ball(true);
    realism_config->set_ball_visibility_threshold(0.4f);
    realism_config->set_camera_overlap(0.3f);
    realism_config->set_dribbler_ball_detections(0);
    realism_config->set_camera_position_error(0);
    realism_config->set_robot_command_loss(0);
    realism_config->set_robot_response_loss(0);
    realism_config->set_missing_ball_detections(0);
    realism_config->set_vision_delay(0);
    realism_config->set_vision_processing_time(0);
    realism_config->set_missing_ball_detections(0);
    realism_config->set_simulate_dribbling(false);
    return realism_config;
}

std::unique_ptr<RealismConfigErForce> ErForceSimulator::createRealisticRealismConfig()
{
    /* values from
     * https://github.com/robotics-erlangen/framework/blob/master/config/simulator-realism/Realistic.txt
     */
    auto realism_config = std::make_unique<RealismConfigErForce>();
    realism_config->set_stddev_ball_p(0.0014f);
    realism_config->set_stddev_robot_p(0.0013f);
    realism_config->set_stddev_robot_phi(0.01f);
    realism_config->set_stddev_ball_area(6.5f);
    realism_config->set_enable_invisible_ball(true);
    realism_config->set_ball_visibility_threshold(0.4f);
    realism_config->set_camera_overlap(1);
    realism_config->set_dribbler_ball_detections(0.05f);
    realism_config->set_camera_position_error(0.1f);
    realism_config->set_robot_command_loss(0.03f);
    realism_config->set_robot_response_loss(0.1f);
    realism_config->set_missing_ball_detections(0.05f);
    realism_config->set_vision_delay(35000000);
    realism_config->set_vision_processing_time(10000000);
    realism_config->set_missing_ball_detections(0.02f);
    realism_config->set_simulate_dribbling(false);
    return realism_config;
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
            auto robot_primitive_executor = std::make_shared<PrimitiveExecutor>(
                Duration::fromSeconds(primitive_executor_time_step), robot_constants,
                TeamColour::BLUE, id);
            blue_primitive_executor_map.insert({id, robot_primitive_executor});
        }
        else
        {
            auto robot_primitive_executor = std::make_shared<PrimitiveExecutor>(
                Duration::fromSeconds(primitive_executor_time_step), robot_constants,
                TeamColour::YELLOW, id);
            yellow_primitive_executor_map.insert({id, robot_primitive_executor});
        }
    }
}

void ErForceSimulator::setYellowRobotPrimitiveSet(
    const TbotsProto::PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::World> world_msg)
{
    ZoneScopedN("setYellowPrimitives");
    // Use chrono to time this function
    static long int total_time_us    = 0;
    static long int num_calls        = 0;
    auto start                       = std::chrono::high_resolution_clock::now();
    auto sim_state                   = getSimulatorState();
    const auto& sim_robots           = sim_state.yellow_robots();
    const auto robot_to_vel_pair_map = getRobotIdToLocalVelocityMap(sim_robots, true);

    yellow_team_world_msg               = std::move(world_msg);
    const TbotsProto::World world_proto = *yellow_team_world_msg;

    std::vector<std::thread> threads;
    for (auto& [robot_id, primitive] : primitive_set_msg.robot_primitives())
    {
                threads.emplace_back([this, robot_id, primitive_set_msg, world_proto,
                                                  &robot_to_vel_pair_map]() {
        auto& [local_vel, angular_vel] = robot_to_vel_pair_map.at(robot_id);
        setRobotPrimitive(robot_id, primitive_set_msg, yellow_primitive_executor_map,
                          world_proto, local_vel, angular_vel);
                });
    }

    for (auto& thread : threads)
    {
        thread.join();
    }

    auto end = std::chrono::high_resolution_clock::now();
    num_calls++;
    total_time_us +=
        std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    if (num_calls % 200 == 0)
    {
        std::cout << "Average time per setYellowRobotPrimitiveSet call: "
                  << total_time_us / num_calls << " us" << std::endl;
        total_time_us = 0;
        num_calls     = 0;
    }
}

void ErForceSimulator::setBlueRobotPrimitiveSet(
    const TbotsProto::PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::World> world_msg)
{
    ZoneScopedN("setBluePrimitives");
    // Use chrono to time this function
    static long int total_time_us    = 0;
    static long int num_calls        = 0;
    auto start                       = std::chrono::high_resolution_clock::now();
    auto sim_state                   = getSimulatorState();
    const auto& sim_robots           = sim_state.blue_robots();
    const auto robot_to_vel_pair_map = getRobotIdToLocalVelocityMap(sim_robots, true);

    blue_team_world_msg                 = std::move(world_msg);
    const TbotsProto::World world_proto = *blue_team_world_msg;

    std::vector<std::thread> threads;
    for (auto& [robot_id, primitive] : primitive_set_msg.robot_primitives())
    {
        threads.emplace_back([this, robot_id, primitive_set_msg, world_proto,
                                          &robot_to_vel_pair_map]() {
        auto& [local_vel, angular_vel] = robot_to_vel_pair_map.at(robot_id);
        setRobotPrimitive(robot_id, primitive_set_msg, blue_primitive_executor_map,
                          world_proto, local_vel, angular_vel);
        });
    }

    for (auto& thread : threads)
    {
        thread.join();
    }

    auto end = std::chrono::high_resolution_clock::now();
    num_calls++;
    total_time_us +=
        std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    if (num_calls % 200 == 0)
    {
        std::cout << "Average time per setYellowRobotPrimitiveSet call: "
                  << total_time_us / num_calls << " us" << std::endl;
        total_time_us = 0;
        num_calls     = 0;
    }
}

void ErForceSimulator::setRobotPrimitive(
    RobotId id, const TbotsProto::PrimitiveSet& primitive_set_msg,
    std::unordered_map<unsigned int, std::shared_ptr<PrimitiveExecutor>>&
        robot_primitive_executor_map,
    const TbotsProto::World& world_msg, const Vector& local_velocity,
    const AngularVelocity angular_velocity)
{
    ZoneScopedN("setRobotPrimitive");
    // Set to NEG_X because the world msg in this simulator is normalized
    // correctly
    auto robot_primitive_executor_iter = robot_primitive_executor_map.find(id);

    if (robot_primitive_executor_iter != robot_primitive_executor_map.end())
    {
        auto primitive_executor = robot_primitive_executor_iter->second;
        primitive_executor->updatePrimitiveSet(primitive_set_msg);
        primitive_executor->updateWorld(world_msg);
        primitive_executor->updateVelocity(local_velocity, angular_velocity);
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
    const TbotsProto::World& world_msg, gameController::Team side)
{
    SSLSimulationProto::RobotControl robot_control;

    auto sim_state = getSimulatorState();
    std::map<RobotId, std::pair<Vector, Angle>> current_velocity_map;
    if (side == gameController::Team::BLUE)
    {
        const auto& sim_robots = sim_state.blue_robots();
        current_velocity_map   = getRobotIdToLocalVelocityMap(sim_robots, false);
    }
    else
    {
        const auto& sim_robots = sim_state.yellow_robots();
        current_velocity_map   = getRobotIdToLocalVelocityMap(sim_robots, false);
    }

    for (auto& primitive_executor_with_id : robot_primitive_executor_map)
    {
        unsigned int robot_id    = primitive_executor_with_id.first;
        auto& primitive_executor = primitive_executor_with_id.second;
        std::unique_ptr<TbotsProto::DirectControlPrimitive> direct_control;

        if (ramping)
        {
            auto direct_control_no_ramp = primitive_executor->stepPrimitive();
            direct_control              = getRampedVelocityPrimitive(
                current_velocity_map.at(robot_id).first,
                current_velocity_map.at(robot_id).second, *direct_control_no_ramp,
                primitive_executor_time_step);
        }
        else
        {
            direct_control = primitive_executor->stepPrimitive();
        }

        auto command = *getRobotCommandFromDirectControl(
            robot_id, std::move(direct_control), robot_constants);
        *(robot_control.mutable_robot_commands()->Add()) = command;
    }
    return robot_control;
}

std::unique_ptr<TbotsProto::DirectControlPrimitive>
ErForceSimulator::getRampedVelocityPrimitive(
    const Vector current_local_velocity,
    const AngularVelocity current_local_angular_velocity,
    TbotsProto::DirectControlPrimitive& target_velocity_primitive,
    const double& time_to_ramp)
{
    TbotsProto::MotorControl_DirectVelocityControl direct_velocity =
        target_velocity_primitive.motor_control().direct_velocity_control();

    // getting the target wheel velocity
    EuclideanSpace_t target_euclidean_velocity = {
        -direct_velocity.velocity().y_component_meters(),
        direct_velocity.velocity().x_component_meters(),
        direct_velocity.angular_velocity().radians_per_second()};

    WheelSpace_t target_wheel_velocity =
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);

    // getting the current wheel velocity
    EuclideanSpace_t current_euclidean_velocity = {
        -current_local_velocity.y(), current_local_velocity.x(),
        current_local_angular_velocity.toRadians()};

    WheelSpace_t current_wheel_velocity =
        euclidean_to_four_wheel.getWheelVelocity(current_euclidean_velocity);

    WheelSpace_t ramped_four_wheel = euclidean_to_four_wheel.rampWheelVelocity(
        current_wheel_velocity, target_wheel_velocity, time_to_ramp);

    EuclideanSpace_t ramped_euclidean =
        euclidean_to_four_wheel.getEuclideanVelocity(ramped_four_wheel);

    auto mutable_direct_velocity = target_velocity_primitive.mutable_motor_control()
                                       ->mutable_direct_velocity_control();
    *(mutable_direct_velocity->mutable_velocity()) =
        *createVectorProto({ramped_euclidean[1], -ramped_euclidean[0]});
    *(mutable_direct_velocity->mutable_angular_velocity()) =
        *createAngularVelocityProto(AngularVelocity::fromRadians(ramped_euclidean[2]));

    return std::make_unique<TbotsProto::DirectControlPrimitive>(
        target_velocity_primitive);
}

void ErForceSimulator::stepSimulation(const Duration& time_step)
{
    current_time = current_time + time_step;

    SSLSimulationProto::RobotControl yellow_robot_control =
        updateSimulatorRobots(yellow_primitive_executor_map, *yellow_team_world_msg,
                              gameController::Team::YELLOW);

    SSLSimulationProto::RobotControl blue_robot_control = updateSimulatorRobots(
        blue_primitive_executor_map, *blue_team_world_msg, gameController::Team::BLUE);

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
    auto robot_status = TbotsProto::RobotStatus();
    auto power_status = TbotsProto::PowerStatus();

    if (blue_robot_with_ball.has_value())
    {
        robot_status.set_robot_id(blue_robot_with_ball.value());
        power_status.set_breakbeam_tripped(true);
    }
    else
    {
        robot_status.clear_robot_id();
        power_status.set_breakbeam_tripped(false);
    }

    *(robot_status.mutable_power_status()) = power_status;
    robot_statuses.push_back(robot_status);

    return robot_statuses;
}

std::vector<TbotsProto::RobotStatus> ErForceSimulator::getYellowRobotStatuses() const
{
    std::vector<TbotsProto::RobotStatus> robot_statuses;
    auto robot_status = TbotsProto::RobotStatus();
    auto power_status = TbotsProto::PowerStatus();

    if (yellow_robot_with_ball.has_value())
    {
        robot_status.set_robot_id(yellow_robot_with_ball.value());
        power_status.set_breakbeam_tripped(true);
    }
    else
    {
        robot_status.clear_robot_id();
        power_status.set_breakbeam_tripped(false);
    }

    *(robot_status.mutable_power_status()) = power_status;
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

std::map<RobotId, std::pair<Vector, AngularVelocity>>
ErForceSimulator::getRobotIdToLocalVelocityMap(const google::protobuf::RepeatedPtrField<world::SimRobot> &sim_robots,
                                               bool plot)
{
    std::map<std::string, double> plot_juggler_values;
    std::map<RobotId, std::pair<Vector, AngularVelocity>> robot_to_local_velocity;
    for (const auto& sim_robot : sim_robots)
    {
        const Vector local_vel =
            globalToLocalVelocity(Vector(sim_robot.v_x(), sim_robot.v_y()),
                                  Angle::fromRadians(sim_robot.angle()));
        const AngularVelocity angular_vel       = Angle::fromRadians(sim_robot.r_z());
        robot_to_local_velocity[sim_robot.id()] = {local_vel, angular_vel};

        plot_juggler_values[std::to_string(sim_robot.id()) + "_sim_vx"] = sim_robot.v_x();
        plot_juggler_values[std::to_string(sim_robot.id()) + "_sim_vy"] = sim_robot.v_y();
        plot_juggler_values[std::to_string(sim_robot.id()) + "_sim_px"] = sim_robot.p_x();
        plot_juggler_values[std::to_string(sim_robot.id()) + "_sim_py"] = sim_robot.p_y();
    }
    if (plot)
    {
        LOG(PLOTJUGGLER) << *createPlotJugglerValue(plot_juggler_values);
    }
    return robot_to_local_velocity;
}
