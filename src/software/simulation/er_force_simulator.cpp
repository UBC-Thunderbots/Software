#include "software/simulation/er_force_simulator.h"

#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>

#include <iostream>

#include "extlibs/er_force_sim/src/protobuf/robot.h"
#include "proto/message_translation/ssl_detection.h"
#include "proto/message_translation/ssl_geometry.h"
#include "proto/message_translation/ssl_simulation_robot_control.h"
#include "proto/message_translation/ssl_wrapper.h"
#include "proto/message_translation/tbots_geometry.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/robot_status_msg.pb.h"
#include "shared/constants.h"
#include "software/embedded/services/imu.h"
#include "software/logger/logger.h"
#include "software/physics/velocity_conversion_util.h"
#include "software/world/robot_state.h"

namespace
{
double sampleGaussianNoise(std::mt19937& rng, double variance)
{
    std::normal_distribution<double> distribution(0.0, std::sqrt(variance));
    return distribution(rng);
}

// Most of a synthesized sensor channel's assumed variance is modeled as a slowly
// drifting bias (an Ornstein-Uhlenbeck process) rather than fresh white noise, since
// real error sources like wheel slip or calibration drift persist over time instead of
// resetting every sample; the rest is left as fast white noise for sample-to-sample
// jitter.
constexpr double BIAS_VARIANCE_FRACTION     = 0.9;
constexpr double BIAS_TIME_CONSTANT_SECONDS = 0.5;

// IMU/motor noise is scaled up from the filter's own assumed variance so the
// synthesized sensors show a visible, meaningful divergence from ground truth instead
// of being dominated by (real, correct) vision corrections.
constexpr double IMU_MOTOR_NOISE_SCALE_FACTOR = 3.0;

// Advances a single drifting bias value by one Euler-Maruyama step of an
// Ornstein-Uhlenbeck process, whose stationary variance equals `stationary_variance`
// and whose fluctuations decorrelate over roughly `BIAS_TIME_CONSTANT_SECONDS`.
void stepDriftingBias(std::mt19937& rng, double& bias, double dt_seconds,
                      double stationary_variance)
{
    const double mean_reversion_rate = 1.0 / BIAS_TIME_CONSTANT_SECONDS;
    const double diffusion_coefficient =
        std::sqrt(2.0 * mean_reversion_rate * stationary_variance);
    std::normal_distribution<double> distribution(0.0, 1.0);
    bias += -mean_reversion_rate * bias * dt_seconds +
            diffusion_coefficient * std::sqrt(dt_seconds) * distribution(rng);
}

// Combines a channel's drifting bias with a smaller fresh white-noise component, both
// drawn from the same total variance per BIAS_VARIANCE_FRACTION.
double sampleCorrelatedNoise(std::mt19937& rng, double& bias, double dt_seconds,
                             double total_variance)
{
    stepDriftingBias(rng, bias, dt_seconds, BIAS_VARIANCE_FRACTION * total_variance);
    return bias +
          sampleGaussianNoise(rng, (1.0 - BIAS_VARIANCE_FRACTION) * total_variance);
}
}  // namespace

const std::string ErForceSimulator::CSV_OUTPUT_PATH = "/tmp/master_test_new.csv";

ErForceSimulator::ErForceSimulator(const TbotsProto::FieldType& field_type,
                                   const robot_constants::RobotConstants& robot_constants,
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
      ramping(ramping),
      noise_rng_(std::random_device{}())
{
    robot_localizer_csv_.open(CSV_OUTPUT_PATH);
    robot_localizer_csv_ << "team,robot_id,estimated_x,actual_x,estimated_y,actual_y,"
                            "estimated_vel_x,actual_vel_x,estimated_vel_y,actual_vel_y\n";
    LOG(INFO) << "Logging RobotLocalizer estimate-vs-ground-truth data to "
              << CSV_OUTPUT_PATH;

    std::string full_filename = CONFIG_DIRECTORY;

    if (field_type == TbotsProto::FieldType::DIV_A)
    {
        full_filename = full_filename + CONFIG_FILE + ".txt";
    }
    else
    {
        // loading division B configuration
        full_filename = full_filename + CONFIG_FILE + "B.txt";
    }

    std::ifstream config_file(full_filename);
    if (!config_file)
    {
        LOG(FATAL) << "Could not open configuration file " << full_filename;
    }

    // Read in the config file
    std::stringstream config_ss;
    config_ss << config_file.rdbuf();
    std::string config_str = config_ss.str();

    google::protobuf::TextFormat::Parser parser;
    std::ignore  = parser.ParseFromString(config_str, &er_force_sim_setup);
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

    if (world_state.has_blue_robots())
    {
        setRobots(world_state.blue_robots().robot_states(), gameController::Team::BLUE);
    }
    if (world_state.has_yellow_robots())
    {
        setRobots(world_state.yellow_robots().robot_states(),
                  gameController::Team::YELLOW);
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
                robot_constants, id, TeamColour::BLUE);
            blue_primitive_executor_map.insert({id, robot_primitive_executor});
        }
        else
        {
            auto robot_primitive_executor = std::make_shared<PrimitiveExecutor>(
                robot_constants, id, TeamColour::YELLOW);
            yellow_primitive_executor_map.insert({id, robot_primitive_executor});
        }
    }
}

void ErForceSimulator::setYellowRobotPrimitiveSet(
    const TbotsProto::PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::World> world_msg)
{
    auto sim_state         = getSimulatorState();
    const auto& sim_robots = sim_state.yellow_robots();
    const auto robot_map =
        getRobotIdToRobotStateMap(sim_robots, gameController::Team::YELLOW);

    yellow_team_world_msg = std::move(world_msg);
    for (auto& [robot_id, primitive] : primitive_set_msg.robot_primitives())
    {
        if (robot_map.contains(robot_id))
        {
            setRobotPrimitive(robot_id, primitive_set_msg, yellow_primitive_executor_map,
                              robot_map.at(robot_id));
            updateLocalizerVisionFromPrimitive(robot_id, primitive, yellow_localizer_map);
        }
    }
}

void ErForceSimulator::setBlueRobotPrimitiveSet(
    const TbotsProto::PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::World> world_msg)
{
    auto sim_state         = getSimulatorState();
    const auto& sim_robots = sim_state.blue_robots();
    const auto robot_map =
        getRobotIdToRobotStateMap(sim_robots, gameController::Team::BLUE);

    blue_team_world_msg = std::move(world_msg);
    for (auto& [robot_id, primitive] : primitive_set_msg.robot_primitives())
    {
        if (robot_map.contains(robot_id))
        {
            setRobotPrimitive(robot_id, primitive_set_msg, blue_primitive_executor_map,
                              robot_map.at(robot_id));
            updateLocalizerVisionFromPrimitive(robot_id, primitive, blue_localizer_map);
        }
    }
}

void ErForceSimulator::setRobotPrimitive(
    RobotId id, const TbotsProto::PrimitiveSet& primitive_set_msg,
    std::unordered_map<unsigned int, std::shared_ptr<PrimitiveExecutor>>&
        robot_primitive_executor_map,
    const RobotState& robot_state)
{
    auto robot_primitive_executor_iter = robot_primitive_executor_map.find(id);

    if (robot_primitive_executor_iter != robot_primitive_executor_map.end())
    {
        auto primitive_executor = robot_primitive_executor_iter->second;
        TbotsProto::RobotStatus robot_status;
        primitive_executor->updateRobotState(robot_state);
        primitive_executor->updatePrimitive(primitive_set_msg.robot_primitives().at(id),
                                            robot_status);
    }
    else
    {
        LOG(WARNING) << "Primitive Executor for robot with ID " << id << " not found"
                     << std::endl;
    }
}

void ErForceSimulator::updateLocalizerVisionFromPrimitive(
    RobotId id, const TbotsProto::Primitive& primitive,
    std::unordered_map<RobotId, SimulatedLocalization>& localizer_map)
{
    if (!primitive.has_move())
    {
        return;
    }

    auto localizer_it = localizer_map.find(id);
    if (localizer_it == localizer_map.end())
    {
        return;
    }

    const Point position =
        createPoint(primitive.move().xy_traj_params().start_position());
    const Angle orientation =
        createAngle(primitive.move().w_traj_params().start_angle());

    localizer_it->second.localizer->update(
        RobotLocalizer::VisionData{position, orientation, RTT_S / 2});
}

SSLSimulationProto::RobotControl ErForceSimulator::updateSimulatorRobots(
    std::unordered_map<unsigned int, std::shared_ptr<PrimitiveExecutor>>&
        robot_primitive_executor_map,
    const TbotsProto::World& world_msg, const Duration& time_step,
    gameController::Team side)
{
    SSLSimulationProto::RobotControl robot_control;

    auto sim_state         = getSimulatorState();
    const auto& sim_robots = (side == gameController::Team::BLUE)
                                 ? sim_state.blue_robots()
                                 : sim_state.yellow_robots();
    const auto robot_map   = getRobotIdToRobotStateMap(sim_robots, side);

    const TeamColour team_colour =
        (side == gameController::Team::BLUE) ? TeamColour::BLUE : TeamColour::YELLOW;
    auto& localizer_map = (side == gameController::Team::BLUE) ? blue_localizer_map
                                                                : yellow_localizer_map;
    updateRobotLocalizers(localizer_map, robot_map, time_step, team_colour);

    for (auto& [robot_id, primitive_executor] : robot_primitive_executor_map)
    {
        std::unique_ptr<TbotsProto::DirectControlPrimitive> direct_control;

        TbotsProto::RobotStatus robot_status;
        if (ramping)
        {
            auto direct_control_no_ramp =
                primitive_executor->stepPrimitive(robot_status, time_step);

            auto* prev_ramp_velocities = &yellow_prev_ramp_velocities;
            if (side == gameController::Team::BLUE)
            {
                prev_ramp_velocities = &blue_prev_ramp_velocities;
            }
            auto prev_it = prev_ramp_velocities->find(robot_id);
            if (prev_it == prev_ramp_velocities->end())
            {
                LocalVelocity seed{Vector(0, 0), AngularVelocity::zero()};
                auto robot_state_it = robot_map.find(robot_id);
                if (robot_state_it != robot_map.end())
                {
                    seed = LocalVelocity{robot_state_it->second.localVelocity(),
                                         robot_state_it->second.angularVelocity()};
                }
                prev_it = prev_ramp_velocities->insert({robot_id, seed}).first;
            }

            direct_control = getRampedVelocityPrimitive(
                prev_it->second.linear, prev_it->second.angular, direct_control_no_ramp,
                time_step);

            // Persist the ramped command as the setpoint to ramp from next tick.
            const auto& ramped =
                direct_control->motor_control().direct_velocity_control();
            prev_it->second =
                LocalVelocity{Vector(ramped.velocity().x_component_meters(),
                                     ramped.velocity().y_component_meters()),
                              AngularVelocity::fromRadians(
                                  ramped.angular_velocity().radians_per_second())};
        }
        else
        {
            direct_control = std::make_unique<TbotsProto::DirectControlPrimitive>(
                primitive_executor->stepPrimitive(robot_status, time_step));
        }

        auto command = *getRobotCommandFromDirectControl(
            robot_id, std::move(direct_control), robot_constants);
        *(robot_control.mutable_robot_commands()->Add()) = command;
    }
    return robot_control;
}

void ErForceSimulator::updateRobotLocalizers(
    std::unordered_map<RobotId, SimulatedLocalization>& localizer_map,
    const std::map<RobotId, RobotState>& robot_map, const Duration& time_step,
    TeamColour team_colour)
{
    const std::string plotjuggler_tag =
        (team_colour == TeamColour::BLUE) ? "_blue_estimated" : "_yellow_estimated";

    for (const auto& [robot_id, ground_truth] : robot_map)
    {
        auto localizer_it = localizer_map.find(robot_id);
        if (localizer_it == localizer_map.end())
        {
            auto localizer =
                std::make_shared<RobotLocalizer>(RobotLocalizer::RobotLocalizerConfig{
                    robot_constants.kalman_process_noise_variance_rad_per_s_4,
                    robot_constants.kalman_vision_noise_variance_rad_2,
                    robot_constants.kalman_motor_sensor_noise_variance_rad_per_s_2});
            localizer_it =
                localizer_map.insert({robot_id, SimulatedLocalization{localizer,
                                                                      SensorBias{}}})
                    .first;
        }
        SimulatedLocalization& localization = localizer_it->second;
        RobotLocalizer& localizer           = *localization.localizer;
        SensorBias& bias                    = localization.bias;

        const double motor_variance =
            IMU_MOTOR_NOISE_SCALE_FACTOR *
            robot_constants.kalman_motor_sensor_noise_variance_rad_per_s_2;
        const double imu_variance =
            IMU_MOTOR_NOISE_SCALE_FACTOR * ImuService::IMU_VARIANCE;
        const double dt_seconds = time_step.toSeconds();

        // IMU: noisy angular velocity, scaled up from the filter's own assumed
        // variance (see IMU_MOTOR_NOISE_SCALE_FACTOR).
        localizer.update(RobotLocalizer::ImuData{
            ground_truth.angularVelocity() +
            AngularVelocity::fromRadians(sampleCorrelatedNoise(
                noise_rng_, bias.imu_angular_velocity, dt_seconds, imu_variance))});

        // Motor sensors: noisy global-frame velocity (ground truth velocity() is
        // already global, so no local<->global conversion is needed here, unlike real
        // Thunderloop, which converts a local motor reading into global using the
        // filter's own orientation estimate).
        const Vector motor_velocity_noise(
            sampleCorrelatedNoise(noise_rng_, bias.motor_velocity_x, dt_seconds,
                                 motor_variance),
            sampleCorrelatedNoise(noise_rng_, bias.motor_velocity_y, dt_seconds,
                                 motor_variance));
        localizer.update(RobotLocalizer::MotorData{
            ground_truth.velocity() + motor_velocity_noise,
            ground_truth.angularVelocity() +
                AngularVelocity::fromRadians(sampleCorrelatedNoise(
                    noise_rng_, bias.motor_angular_velocity, dt_seconds,
                    motor_variance))});

        // Predict step: matches real Thunderloop, which currently passes a zero
        // control input (see RobotLocalizer::step call in thunderloop.cpp). Using a
        // ground-truth-derived acceleration here instead would give the filter a
        // noise-free "cheat" channel to fall back on whenever it distrusts the
        // (deliberately noisy) measurements, undermining the whole point of this
        // side-channel comparison.
        localizer.step(Vector(), time_step);

        // Vision is NOT synthesized here - see updateLocalizerVisionFromPrimitive(),
        // which feeds this localizer the actual vision-derived position the AI used
        // to plan this robot's trajectory, whenever a new primitive arrives.

        RobotLocalizer::logToPlotJuggler(robot_id, localizer.getRobotState(),
                                         plotjuggler_tag);

        robot_localizer_csv_ << (team_colour == TeamColour::BLUE ? "blue" : "yellow")
                             << ',' << robot_id << ',' << localizer.getPosition().x()
                             << ',' << ground_truth.position().x() << ','
                             << localizer.getPosition().y() << ','
                             << ground_truth.position().y() << ','
                             << localizer.getVelocity().x() << ','
                             << ground_truth.velocity().x() << ','
                             << localizer.getVelocity().y() << ','
                             << ground_truth.velocity().y() << '\n';
    }
}

std::unique_ptr<TbotsProto::DirectControlPrimitive>
ErForceSimulator::getRampedVelocityPrimitive(
    const Vector current_local_velocity,
    const AngularVelocity current_local_angular_velocity,
    TbotsProto::DirectControlPrimitive& target_velocity_primitive, Duration time_to_ramp)
{
    TbotsProto::MotorControl_DirectVelocityControl direct_velocity =
        target_velocity_primitive.motor_control().direct_velocity_control();

    // getting the target wheel velocity
    EuclideanSpace_t target_euclidean_velocity = {
        direct_velocity.velocity().x_component_meters(),
        direct_velocity.velocity().y_component_meters(),
        direct_velocity.angular_velocity().radians_per_second()};

    WheelSpace_t target_wheel_velocity =
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);

    // getting the current wheel velocity
    EuclideanSpace_t current_euclidean_velocity = {
        current_local_velocity.x(), current_local_velocity.y(),
        current_local_angular_velocity.toRadians()};

    WheelSpace_t current_wheel_velocity =
        euclidean_to_four_wheel.getWheelVelocity(current_euclidean_velocity);

    WheelSpace_t ramped_four_wheel = euclidean_to_four_wheel.rampWheelVelocity(
        current_wheel_velocity, target_wheel_velocity, time_to_ramp.toSeconds());

    EuclideanSpace_t ramped_euclidean =
        euclidean_to_four_wheel.getEuclideanVelocity(ramped_four_wheel);

    auto mutable_direct_velocity = target_velocity_primitive.mutable_motor_control()
                                       ->mutable_direct_velocity_control();
    *(mutable_direct_velocity->mutable_velocity()) =
        *createVectorProto({ramped_euclidean[0], ramped_euclidean[1]});
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
                              time_step, gameController::Team::YELLOW);

    SSLSimulationProto::RobotControl blue_robot_control =
        updateSimulatorRobots(blue_primitive_executor_map, *blue_team_world_msg,
                              time_step, gameController::Team::BLUE);

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

std::map<RobotId, RobotState> ErForceSimulator::getRobotIdToRobotStateMap(
    const google::protobuf::RepeatedPtrField<world::SimRobot>& sim_robots,
    gameController::Team side)
{
    std::map<RobotId, RobotState> robot_map;
    for (const auto& sim_robot : sim_robots)
    {
        auto position         = Point(sim_robot.p_x(), sim_robot.p_y());
        auto velocity         = Vector(sim_robot.v_x(), sim_robot.v_y());
        auto orientation      = Angle::fromRadians(sim_robot.angle());
        auto angular_velocity = AngularVelocity::fromRadians(sim_robot.r_z());

        if (side == gameController::Team::YELLOW)
        {
            position = -position;
            velocity = -velocity;
            orientation += Angle::half();
            // angular_velocity is the same no matter which side
        }

        robot_map[sim_robot.id()] =
            RobotState(position, velocity, orientation, angular_velocity);
    }
    return robot_map;
}
