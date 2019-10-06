#include "software/backend/simulator_backend.h"
#include "software/backend/backend_factory.h"

const std::string SimulatorBackend::name = "simulator";

SimulatorBackend::SimulatorBackend(const Duration &simulation_time_step, unsigned int num_steps_per_primitive_update) :
        simulation_time_step(simulation_time_step), num_steps_per_primitive_update(num_steps_per_primitive_update)
{
}

void SimulatorBackend::onValueReceived(ConstPrimitiveVectorPtr primitives)
{
    most_recently_received_primitives_mutex.lock();
    most_recently_received_primitives = std::move(primitives);
    most_recently_received_primitives_mutex.unlock();
    updateSimulation();
}

void SimulatorBackend::setWorld(const World &new_world) {
    // TODO: Set up simulation world (#768)

    // We need to update the simulation (which sends the World to observers)
    // when we set a new world to make sure we start the feedback loop between
    // this simulator backend and the module that observes the World and produces
    // primitives
    updateSimulation();
}

World SimulatorBackend::getWorld() {
    std::scoped_lock world_lock(world_mutex);
    return world;
}

void SimulatorBackend::updateSimulation() {
    for(unsigned int i = 0; i < num_steps_per_primitive_update; i++) {
        // TODO: update sim with time_step (#768)
    }

    // TODO: update world with simulated world (#768)

    sendWorldToObservers();
}

void SimulatorBackend::sendWorldToObservers() {
    std::scoped_lock world_lock(world_mutex);
    Subject<World>::sendValueToObservers(world);
}

// Register this backend in the BackendFactory
static TBackendFactory<SimulatorBackend> factory;
