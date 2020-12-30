#include "software/simulation/simulator.h"

#include "software/proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "software/proto/message_translation/ssl_detection.h"
#include "software/proto/message_translation/ssl_geometry.h"
#include "software/proto/message_translation/ssl_wrapper.h"
#include "software/simulation/simulator_ball_singleton.h"
#include "software/simulation/simulator_robot_singleton.h"

extern "C"
{
#include "firmware/app/logger/logger.h"
#include "firmware/app/world/firmware_ball.h"
#include "firmware/app/world/firmware_robot.h"
#include "firmware/app/world/firmware_world.h"
#include "shared/proto/robot_log_msg.nanopb.h"
}

Simulator::Simulator(const Field& field,
                     std::shared_ptr<const SimulatorConfig> simulator_config,
                     const Duration& physics_time_step)
    : physics_world(field, simulator_config),
      yellow_team_defending_side(FieldSide::NEG_X),
      blue_team_defending_side(FieldSide::NEG_X),
      frame_number(0),
      physics_time_step(physics_time_step)
{
}

void Simulator::setBallState(const BallState& ball_state)
{
    physics_world.setBallState(ball_state);
    simulator_ball = std::make_shared<SimulatorBall>(physics_world.getPhysicsBall());

    for (auto& robot_pair : yellow_simulator_robots)
    {
        robot_pair.first->clearBallInDribblerArea();
    }

    for (auto& robot_pair : blue_simulator_robots)
    {
        robot_pair.first->clearBallInDribblerArea();
    }
}

void Simulator::removeBall()
{
    simulator_ball.reset();
    physics_world.removeBall();
}

void Simulator::addYellowRobots(const std::vector<RobotStateWithId>& robots)
{
    physics_world.addYellowRobots(robots);
    updateSimulatorRobots(physics_world.getYellowPhysicsRobots(), yellow_simulator_robots,
                          TeamColour::YELLOW);
}

void Simulator::addBlueRobots(const std::vector<RobotStateWithId>& robots)
{
    physics_world.addBlueRobots(robots);
    updateSimulatorRobots(physics_world.getBluePhysicsRobots(), blue_simulator_robots,
                          TeamColour::BLUE);
}

void Simulator::updateSimulatorRobots(
    const std::vector<std::weak_ptr<PhysicsRobot>>& physics_robots,
    std::map<std::shared_ptr<SimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>&
        simulator_robots,
    TeamColour team_colour)
{
    for (const auto& physics_robot : physics_robots)
    {
        auto simulator_robot = std::make_shared<SimulatorRobot>(physics_robot);

        // we initialize the logger with the appropriate logging function based
        // on the team color and the robot id to propagate any logs when creating
        // the firmware_robot and firmware_ball
        if (team_colour == TeamColour::BLUE)
        {
            app_logger_init(simulator_robot->getRobotId(),
                            &SimulatorRobotSingleton::handleBlueRobotLogProto);
        }
        else if (team_colour == TeamColour::YELLOW)
        {
            app_logger_init(simulator_robot->getRobotId(),
                            &SimulatorRobotSingleton::handleYellowRobotLogProto);
        }

        auto firmware_robot = SimulatorRobotSingleton::createFirmwareRobot();
        auto firmware_ball  = SimulatorBallSingleton::createFirmwareBall();

        FirmwareWorld_t* firmware_world_raw =
            app_firmware_world_create(firmware_robot.release(), firmware_ball.release(),
                                      &(Simulator::getCurrentFirmwareTimeSeconds));
        auto firmware_world =
            std::shared_ptr<FirmwareWorld_t>(firmware_world_raw, FirmwareWorldDeleter());

        simulator_robots.insert(std::make_pair(simulator_robot, firmware_world));
    }
}

void Simulator::setYellowRobotPrimitive(RobotId id,
                                        const TbotsProto_Primitive& primitive_msg)
{
    setRobotPrimitive(id, primitive_msg, yellow_simulator_robots, simulator_ball,
                      yellow_team_defending_side);
}

void Simulator::setBlueRobotPrimitive(RobotId id,
                                      const TbotsProto_Primitive& primitive_msg)
{
    setRobotPrimitive(id, primitive_msg, blue_simulator_robots, simulator_ball,
                      blue_team_defending_side);
}

void Simulator::setYellowRobotPrimitiveSet(
    const TbotsProto_PrimitiveSet& primitive_set_msg)
{
    for (pb_size_t i = 0; i < primitive_set_msg.robot_primitives_count; i++)
    {
        setYellowRobotPrimitive(primitive_set_msg.robot_primitives[i].key,
                                primitive_set_msg.robot_primitives[i].value);
    }
}

void Simulator::setBlueRobotPrimitiveSet(const TbotsProto_PrimitiveSet& primitive_set_msg)
{
    for (pb_size_t i = 0; i < primitive_set_msg.robot_primitives_count; i++)
    {
        setBlueRobotPrimitive(primitive_set_msg.robot_primitives[i].key,
                              primitive_set_msg.robot_primitives[i].value);
    }
}

void Simulator::setRobotPrimitive(
    RobotId id, const TbotsProto_Primitive& primitive_msg,
    std::map<std::shared_ptr<SimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>&
        simulator_robots,
    const std::shared_ptr<SimulatorBall>& simulator_ball, FieldSide defending_side)
{
    SimulatorBallSingleton::setSimulatorBall(simulator_ball, defending_side);
    auto simulator_robots_iter =
        std::find_if(simulator_robots.begin(), simulator_robots.end(),
                     [id](const auto& robot_world_pair) {
                         return robot_world_pair.first->getRobotId() == id;
                     });

    if (simulator_robots_iter != simulator_robots.end())
    {
        auto simulator_robot = (*simulator_robots_iter).first;
        auto firmware_world  = (*simulator_robots_iter).second;
        SimulatorRobotSingleton::setSimulatorRobot(simulator_robot, defending_side);
        SimulatorRobotSingleton::startNewPrimitiveOnCurrentSimulatorRobot(firmware_world,
                                                                          primitive_msg);
    }
}

void Simulator::setYellowTeamDefendingSide(const DefendingSideProto& defending_side_proto)
{
    switch (defending_side_proto.defending_side())
    {
        case DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_NEG_X:
            yellow_team_defending_side = FieldSide::NEG_X;
            break;
        case DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_POS_X:
            yellow_team_defending_side = FieldSide::POS_X;
            break;
        default:
            throw std::invalid_argument(
                "Unhandled value of DefendingSideProto_FieldSide");
    }
}

void Simulator::setBlueTeamDefendingSide(const DefendingSideProto& defending_side_proto)
{
    switch (defending_side_proto.defending_side())
    {
        case DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_NEG_X:
            blue_team_defending_side = FieldSide::NEG_X;
            break;
        case DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_POS_X:
            blue_team_defending_side = FieldSide::POS_X;
            break;
        default:
            throw std::invalid_argument(
                "Unhandled value of DefendingSideProto_FieldSide");
    }
}

void Simulator::stepSimulation(const Duration& time_step)
{
    // Set the ball being referenced in each firmware_world.
    // We only need to do this a single time since all robots
    // can see and interact with the same ball

    Duration remaining_time = time_step;
    while (remaining_time > Duration::fromSeconds(0))
    {
        current_firmware_time = physics_world.getTimestamp();

        for (auto& iter : blue_simulator_robots)
        {
            auto simulator_robot = iter.first;
            auto firmware_world  = iter.second;

            app_logger_init(simulator_robot->getRobotId(),
                            &SimulatorRobotSingleton::handleBlueRobotLogProto);

            SimulatorRobotSingleton::setSimulatorRobot(simulator_robot,
                                                       blue_team_defending_side);
            SimulatorBallSingleton::setSimulatorBall(simulator_ball,
                                                     blue_team_defending_side);
            SimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(firmware_world);
        }

        for (auto& iter : yellow_simulator_robots)
        {
            auto simulator_robot = iter.first;
            auto firmware_world  = iter.second;

            app_logger_init(simulator_robot->getRobotId(),
                            &SimulatorRobotSingleton::handleYellowRobotLogProto);

            SimulatorRobotSingleton::setSimulatorRobot(simulator_robot,
                                                       yellow_team_defending_side);
            SimulatorBallSingleton::setSimulatorBall(simulator_ball,
                                                     yellow_team_defending_side);
            SimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(firmware_world);
        }

        // We take as many steps of `physics_time_step` as possible, and then
        // simulate the remainder of the time
        Duration dt = std::min(remaining_time, physics_time_step);
        physics_world.stepSimulation(dt);
        remaining_time = remaining_time - physics_time_step;
    }

    frame_number++;
}

World Simulator::getWorld() const
{
    Timestamp timestamp = physics_world.getTimestamp();
    // The world currently must contain a ball. The ability to represent no ball
    // will be fixed in https://github.com/UBC-Thunderbots/Software/issues/1325
    Ball ball = Ball(Point(0, 0), Vector(0, 0), timestamp);
    if (physics_world.getBallState())
    {
        ball = Ball(BallState(physics_world.getBallState().value()), timestamp);
    }

    // Note: The simulator currently makes the invariant that friendly robots
    // are yellow robots, and enemies are blue. This will be fixed in
    // https://github.com/UBC-Thunderbots/Software/issues/1325
    std::vector<Robot> friendly_team_robots;
    for (const auto& robot_state : physics_world.getYellowRobotStates())
    {
        Robot robot(robot_state.id, robot_state.robot_state, timestamp);
        friendly_team_robots.emplace_back(robot);
    }
    std::vector<Robot> enemy_team_robots;
    for (const auto& robot_state : physics_world.getBlueRobotStates())
    {
        Robot robot(robot_state.id, robot_state.robot_state, timestamp);
        enemy_team_robots.emplace_back(robot);
    }

    Team friendly_team(friendly_team_robots, Duration::fromSeconds(0.5));
    Team enemy_team(enemy_team_robots, Duration::fromSeconds(0.5));

    World world(physics_world.getField(), ball, friendly_team, enemy_team);
    return world;
}

std::unique_ptr<SSLProto::SSL_WrapperPacket> Simulator::getSSLWrapperPacket() const
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

Field Simulator::getField() const
{
    return physics_world.getField();
}

Timestamp Simulator::getTimestamp() const
{
    return physics_world.getTimestamp();
}

std::weak_ptr<PhysicsRobot> Simulator::getRobotAtPosition(const Point& position)
{
    return physics_world.getRobotAtPosition(position);
}

void Simulator::addYellowRobot(const Point& position)
{
    RobotId id = physics_world.getAvailableYellowRobotId();
    auto state =
        RobotState(position, Vector(0, 0), Angle::zero(), AngularVelocity::zero());
    auto state_with_id = RobotStateWithId{.id = id, .robot_state = state};
    addYellowRobots({state_with_id});
}

void Simulator::addBlueRobot(const Point& position)
{
    RobotId id = physics_world.getAvailableBlueRobotId();
    auto state =
        RobotState(position, Vector(0, 0), Angle::zero(), AngularVelocity::zero());
    auto state_with_id = RobotStateWithId{.id = id, .robot_state = state};
    addBlueRobots({state_with_id});
}

void Simulator::removeRobot(std::weak_ptr<PhysicsRobot> robot)
{
    physics_world.removeRobot(robot);
}

float Simulator::getCurrentFirmwareTimeSeconds()
{
    return static_cast<float>(current_firmware_time.toSeconds());
}

// We must give this variable a value here, as non-const static variables must be
// initialized out-of-line
Timestamp Simulator::current_firmware_time = Timestamp::fromSeconds(0);
