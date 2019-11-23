#include "software/backend/simulator_backend.h"

#include <math.h>

#include <algorithm>

#include "software/util/logger/init.h"

const std::string SimulatorBackend::name = "simulator";

SimulatorBackend::SimulatorBackend(
    const Duration &physics_time_step, const Duration &world_time_increment,
    SimulatorBackend::SimulationSpeed simulation_speed_mode)
    : physics_time_step(physics_time_step),
      world_time_increment(world_time_increment),
      simulation_speed_mode(simulation_speed_mode)
{
}

void SimulatorBackend::onValueReceived(ConstPrimitiveVectorPtr primitives)
{
    primitive_promise.set_value(std::move(primitives));
}

void SimulatorBackend::setSimulationSpeed(
    SimulatorBackend::SimulationSpeed simulation_speed_mode)
{
    this->simulation_speed_mode = simulation_speed_mode;
}

bool SimulatorBackend::runSimulation(World world, const Duration &timeout)
{
    PhysicsSimulator physics_simulator(world);

    Timestamp timeout_timestamp = world.getMostRecentTimestamp() + timeout;
    std::future<ConstPrimitiveVectorPtr> primitive_future =
        primitive_promise.get_future();
    Subject<World>::sendValueToObservers(world);
    while (world.getMostRecentTimestamp() <= timeout_timestamp)
    {
        for (unsigned int i = 0;
             i < static_cast<unsigned int>(std::ceil(world_time_increment.getSeconds() /
                                                     physics_time_step.getSeconds()));
             i++)
        {
            world = physics_simulator.stepSimulation(physics_time_step);
        }

        // TODO: Re-enable with issue #1029
        //        auto status = primitive_future.wait_for(std::chrono::seconds(5));
        //        if(status != std::future_status::ready) {
        //            LOG(WARNING) << "Timed out waiting for primitives. Aborting
        //            simulation..."; return false;
        //        }

        if (simulation_speed_mode == SimulationSpeed::REALTIME_SIMULATION)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                std::lrint(world_time_increment.getMilliseconds())));
        }

        Subject<World>::sendValueToObservers(world);
    }

    LOG(WARNING) << "Simulation timed out";
    return false;
}
