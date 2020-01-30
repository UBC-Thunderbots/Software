#include "software/backend/simulation/simulator.h"
#include "software/backend/simulation/simulator_robot_singleton.h"
#include "software/backend/simulation/simulator_ball_singleton.h"
extern "C"
{
#include "app/world/firmware_world.h"
}

Simulator::Simulator(const World &world) : physics_world(world) {
    for(auto physics_robot : physics_world.getFriendlyPhysicsRobots()) {
        friendly_simulator_robots.emplace_back(SimulatorRobot(physics_robot))
    }

    SimulatorRobotSingleton::setSimulatorRobots(friendly_simulator_robots);
}

void Simulator::stepSimulation(const Duration time_step) {
    auto firmware_robot = SimulatorRobotSingleton::createFirmwareRobot();
    auto firmware_ball = SimulatorBallSingleton::createFirmwareBall();
    auto firmware_world = std::make_unique<FirmwareWorld_t>(app_firmware_world_create(firmware_robot, firmware_ball));
    SimulatorBallSingleton::setPhysicsBall(physics_world.getPhysicsBall()):
    for(const auto& robot : friendly_simulator_robots) {
        SimulatorRobotSingleton::setRobotId(robot.getRobotId());
        // app_main(firmware_world.get(), PRIMITIVE STUFF);
    }

    physics_world.step(time_step):
}

void Simulator::setPrimitives(ConstPrimitiveVectorPtr primitives) {
    for(auto p : *primitives) {
        // find corresponding robot in friendly_simulator_robots
        // robot.setPrimitive(p)
    }
}

World Simulator::getWorld() {
    physics_world.getWorld();
}
