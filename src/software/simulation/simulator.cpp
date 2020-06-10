#include "software/simulation/simulator.h"

#include "software/backend/output/radio/mrf/mrf_primitive_visitor.h"
#include "software/simulation/simulator_ball_singleton.h"
#include "software/simulation/simulator_robot_singleton.h"

extern "C"
{
#include "firmware/app/world/firmware_ball.h"
#include "firmware/app/world/firmware_robot.h"
#include "firmware/app/world/firmware_world.h"
}

Simulator::Simulator(const Field& field) : physics_world(field) {}

void Simulator::setBallState(const BallState &ball_state) {
    physics_world.setBallState(ball_state);
    simulator_ball = std::make_shared<SimulatorBall>(physics_world.getPhysicsBall());
}

void Simulator::removeBall() {
    simulator_ball.reset();
    physics_world.removeBall();
}

void Simulator::addYellowRobots(const std::vector<RobotStateWithId> &robots) {
    physics_world.addYellowRobots(robots);
    for (auto physics_robot : physics_world.getYellowPhysicsRobots())
    {
        auto simulator_robot = std::make_shared<SimulatorRobot>(physics_robot);
        auto firmware_robot = SimulatorRobotSingleton::createFirmwareRobot();
        auto firmware_ball  = SimulatorBallSingleton::createFirmwareBall();
        // Release ownership of the pointers so the firmware_world can take ownership
        FirmwareWorld_t* firmware_world_raw =
            app_firmware_world_create(firmware_robot.release(), firmware_ball.release());
        auto firmware_world =
            std::shared_ptr<FirmwareWorld_t>(firmware_world_raw, FirmwareWorldDeleter());

        yellow_simulator_robots.insert(std::make_pair(simulator_robot, firmware_world));
    }
}

void Simulator::addBlueRobots(const std::vector<RobotStateWithId> &robots) {
    physics_world.addBlueRobots(robots);
    for (auto physics_robot : physics_world.getBluePhysicsRobots())
    {
        auto simulator_robot = std::make_shared<SimulatorRobot>(physics_robot);
        auto firmware_robot = SimulatorRobotSingleton::createFirmwareRobot();
        auto firmware_ball  = SimulatorBallSingleton::createFirmwareBall();
        // Release ownership of the pointers so the firmware_world can take ownership
        FirmwareWorld_t* firmware_world_raw =
                app_firmware_world_create(firmware_robot.release(), firmware_ball.release());
        auto firmware_world =
                std::shared_ptr<FirmwareWorld_t>(firmware_world_raw, FirmwareWorldDeleter());

        blue_simulator_robots.insert(std::make_pair(simulator_robot, firmware_world));
    }
}

void Simulator::setYellowRobotPrimitives(ConstPrimitiveVectorPtr primitives)
{
    if (!primitives)
    {
        return;
    }

    SimulatorBallSingleton::setSimulatorBall(simulator_ball);
    for (const auto& primitive_ptr : *primitives)
    {
        primitive_params_t primitive_params = getPrimitiveParams(primitive_ptr);
        unsigned int primitive_index        = getPrimitiveIndex(primitive_ptr);

        auto simulator_robots_iter =
            std::find_if(yellow_simulator_robots.begin(), yellow_simulator_robots.end(),
                         [&primitive_ptr](const auto& robot_world_pair) {
                             return robot_world_pair.first->getRobotId() ==
                                    primitive_ptr->getRobotId();
                         });

        if (simulator_robots_iter != yellow_simulator_robots.end())
        {
            auto simulator_robot = (*simulator_robots_iter).first;
            auto firmware_world  = (*simulator_robots_iter).second;
            SimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
            SimulatorRobotSingleton::startNewPrimitiveOnCurrentSimulatorRobot(
                firmware_world, primitive_index, primitive_params);
        }
    }
}

void Simulator::setBlueRobotPrimitives(ConstPrimitiveVectorPtr primitives)
{
    if (!primitives)
    {
        return;
    }

    SimulatorBallSingleton::setSimulatorBall(simulator_ball);
    for (const auto& primitive_ptr : *primitives)
    {
        primitive_params_t primitive_params = getPrimitiveParams(primitive_ptr);
        unsigned int primitive_index        = getPrimitiveIndex(primitive_ptr);

        auto simulator_robots_iter =
                std::find_if(blue_simulator_robots.begin(), blue_simulator_robots.end(),
                             [&primitive_ptr](const auto& robot_world_pair) {
                                 return robot_world_pair.first->getRobotId() ==
                                        primitive_ptr->getRobotId();
                             });

        if (simulator_robots_iter != blue_simulator_robots.end())
        {
            auto simulator_robot = (*simulator_robots_iter).first;
            auto firmware_world  = (*simulator_robots_iter).second;
            SimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
            SimulatorRobotSingleton::startNewPrimitiveOnCurrentSimulatorRobot(
                    firmware_world, primitive_index, primitive_params);
        }
    }
}

void Simulator::stepSimulation(const Duration& time_step)
{
    SimulatorBallSingleton::setSimulatorBall(simulator_ball);

    for (auto& iter : yellow_simulator_robots)
    {
        auto simulator_robot = iter.first;
        auto firmware_world  = iter.second;
        SimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
        SimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(firmware_world);
    }

    for (auto& iter : blue_simulator_robots)
    {
        auto simulator_robot = iter.first;
        auto firmware_world  = iter.second;
        SimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
        SimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(firmware_world);
    }

    physics_world.stepSimulation(time_step);
}

World Simulator::getWorld() const
{
    Timestamp timestamp = physics_world.getTimestamp();
    // The world currently must contain a ball. The ability to represent no ball
    // will be fixed in https://github.com/UBC-Thunderbots/Software/issues/1325
    Ball ball = Ball(Point(0, 0), Vector(0, 0), timestamp);
    if (physics_world.getBallState())
    {
        ball =
            Ball(TimestampedBallState(physics_world.getBallState().value(), timestamp));
    }

    // Note: The simulator currently makes the invariant that friendly robots
    // are yellow robots, and enemies are blue. This will be fixed in
    // https://github.com/UBC-Thunderbots/Software/issues/1325
    std::vector<Robot> friendly_team_robots;
    for (const auto& robot_state : physics_world.getYellowRobotStates())
    {
        TimestampedRobotState timestamped_robot_state(robot_state.robot_state, timestamp);
        Robot robot(robot_state.id, timestamped_robot_state);
        friendly_team_robots.emplace_back(robot);
    }
    std::vector<Robot> enemy_team_robots;
    for (const auto& robot_state : physics_world.getBlueRobotStates())
    {
        TimestampedRobotState timestamped_robot_state(robot_state.robot_state, timestamp);
        Robot robot(robot_state.id, timestamped_robot_state);
        enemy_team_robots.emplace_back(robot);
    }

    Team friendly_team(friendly_team_robots, Duration::fromSeconds(0.5));
    Team enemy_team(enemy_team_robots, Duration::fromSeconds(0.5));

    World world(physics_world.getField(), ball, friendly_team, enemy_team);

    // TODO: This is a hack to persist goalie ID from the initial test setup
    // It will be removed as part of
    // https://github.com/UBC-Thunderbots/Software/issues/1325
//    auto id = friendly_goalie_id;
//    if (id)
//    {
//        world.mutableFriendlyTeam().assignGoalie(*id);
//    }
    return world;
}

primitive_params_t Simulator::getPrimitiveParams(
    const std::unique_ptr<Primitive>& primitive)
{
    // The MRFPrimitiveVisitor handles most of the encoding for us
    MRFPrimitiveVisitor mrf_pv;
    primitive->accept(mrf_pv);
    RadioPrimitive_t radio_primitive = mrf_pv.getSerializedRadioPacket();
    primitive_params_t primitive_params;
    std::array<double, 4> param_array = radio_primitive.param_array;
    for (unsigned int i = 0; i < param_array.size(); i++)
    {
        // The data is already scaled appropriately for us from the
        // getSerializedRadioPacket function. We just need to pack it
        // into an int16_t
        double data                = param_array[i];
        primitive_params.params[i] = static_cast<int16_t>(std::round(data));
    }

    primitive_params.slow  = radio_primitive.slow;
    primitive_params.extra = radio_primitive.extra_bits;

    return primitive_params;
}

unsigned int Simulator::getPrimitiveIndex(const std::unique_ptr<Primitive>& primitive)
{
    MRFPrimitiveVisitor mrf_pv;
    primitive->accept(mrf_pv);
    RadioPrimitive_t radio_primitive = mrf_pv.getSerializedRadioPacket();
    auto primitive_index = static_cast<unsigned int>(radio_primitive.prim_type);

    return primitive_index;
}
