#include "software/backend/simulation/simulator.h"
#include "software/backend/simulation/simulator_robot_singleton.h"
#include "software/backend/simulation/simulator_ball_singleton.h"
extern "C"
{
#include "firmware/main/app/world/firmware_world.h"
}

/**
 * Because the FirmwareWorld_t struct is defined in the .c file (rather than the .h file),
 * C++ considers it an incomplete type and is unable to use it with smart pointers
 * because it doesn't know the size of the object. Therefore we need to create our own
 * "Deleter" class we can provide to the smart pointers to handle that instead.
 *
 * See https://en.cppreference.com/w/cpp/memory/unique_ptr/unique_ptr for more info and
 * examples
 */
struct FirmwareWorldDeleter
{
    void operator()(FirmwareWorld_t* firmware_world) const
    {
        app_firmware_world_destroy(firmware_world);
    };
};


Simulator::Simulator(const World &world) : physics_world(world) {
    for(auto physics_robot : physics_world.getFriendlyPhysicsRobots()) {
        auto simulator_robot = std::make_shared<SimulatorRobot>(physics_robot);
        friendly_simulator_robots.emplace_back(simulator_robot);
    }
    simulator_ball = std::make_shared<SimulatorBall>(physics_world.getPhysicsBall());
}

void Simulator::stepSimulation(const Duration& time_step) {
    auto firmware_robot = SimulatorRobotSingleton::createFirmwareRobot();
    auto firmware_ball = SimulatorBallSingleton::createFirmwareBall();
    FirmwareWorld_t* firmware_world_raw = app_firmware_world_create(firmware_robot.get(), firmware_ball.get());
    auto firmware_world = std::unique_ptr<FirmwareWorld_t, FirmwareWorldDeleter>(firmware_world_raw, FirmwareWorldDeleter());
    SimulatorBallSingleton::setSimulatorBall(simulator_ball);
    for(const auto& simulator_robot : friendly_simulator_robots) {
        SimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
        // TODO: Simulate primitives
        // app_main(firmware_world.get(), PRIMITIVE STUFF);
    }

    physics_world.stepSimulation(time_step);
}

void Simulator::setPrimitives(ConstPrimitiveVectorPtr primitives) {
    // TODO: Set primitives
    // https://github.com/UBC-Thunderbots/Software/issues/768
}

World Simulator::getWorld() {
    return physics_world.getWorld();
}
