#include "software/backend/simulator_backend.h"

#include <math.h>

#include <algorithm>

#include "software/backend/backend_factory.h"
#include "software/util/logger/init.h"

const std::string SimulatorBackend::name = "simulator";

SimulatorBackend::SimulatorBackend(
    const Duration &physics_time_step, const Duration &world_time_increment,
    SimulatorBackend::SimulationSpeed simulation_speed_mode)
    : physics_time_step(physics_time_step),
      world_time_increment(world_time_increment),
      simulation_speed_mode(simulation_speed_mode),
      primitive_buffer(primitive_buffer_size),
      in_destructor(false),
      simulation_thread_started(false)
{
}

SimulatorBackend::SimulatorBackend()
    : SimulatorBackend(Duration::fromMilliseconds(5), Duration::fromSeconds(1.0 / 30.0),
                       SimulationSpeed::REALTIME_SIMULATION)
{
}

SimulatorBackend::~SimulatorBackend()
{
    stopSimulation();
}

void SimulatorBackend::onValueReceived(ConstPrimitiveVectorPtr primitives)
{
    primitive_buffer.push(primitives);
}

void SimulatorBackend::setSimulationSpeed(
    SimulatorBackend::SimulationSpeed simulation_speed_mode)
{
    this->simulation_speed_mode = simulation_speed_mode;
}

void SimulatorBackend::startSimulation(World world)
{
    // Start the thread to do the simulation in the background
    // The lambda expression here is needed so that we can call
    // `runSimulationLoop()`, which is not a static function
    simulation_thread_started = true;
    simulation_thread = std::thread(&SimulatorBackend::runSimulationLoop, this, world);
}

void SimulatorBackend::stopSimulation()
{
    if (simulation_thread_started)
    {
        simulation_thread_started = false;

        // Set this flag so the simulation_thread knows to end (also making sure to
        // properly take and give ownership of the flag)
        in_destructor_mutex.lock();
        in_destructor = true;
        in_destructor_mutex.unlock();

        // Join to simulation_thread so that we wait for it to exit before destructing
        // the thread object. If we do not wait for thread to finish executing, it will
        // call `std::terminate` when we deallocate the thread object and kill our whole
        // program
        simulation_thread.join();
    }
}

void SimulatorBackend::runSimulationLoop(World world)
{
    unsigned int num_physics_steps_per_world_published = static_cast<unsigned int>(
        std::ceil(world_time_increment.getSeconds() / physics_time_step.getSeconds()));

    PhysicsSimulator physics_simulator(world);

    auto world_publish_timestamp = std::chrono::steady_clock::now();

    // Take ownership of the in_destructor flag so we can use it for the conditional
    // check
    in_destructor_mutex.lock();
    while (!in_destructor)
    {
        // Give up ownership of the in_destructor flag now that we're done the
        // conditional check
        in_destructor_mutex.unlock();

        for (unsigned int i = 0; i < num_physics_steps_per_world_published; i++)
        {
            world = physics_simulator.stepSimulation(physics_time_step);
        }

        if (simulation_speed_mode.load() == SimulationSpeed::REALTIME_SIMULATION)
        {
            // Calculate how much wall-clock time has passed since we last published a
            // world, and sleep for as much time as necessary for it to have been
            // world_time_increment seconds in wall-clock time since the last world was
            // published.
            auto timestamp_now = std::chrono::steady_clock::now();
            auto milliseconds_since_world_publish =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    timestamp_now - world_publish_timestamp);
            auto remaining_milliseconds = std::chrono::milliseconds(std::lrint(
                                              world_time_increment.getMilliseconds())) -
                                          milliseconds_since_world_publish;

            if (remaining_milliseconds > std::chrono::milliseconds(0))
            {
                std::this_thread::sleep_for(remaining_milliseconds);
            }
        }

        Subject<World>::sendValueToObservers(world);

        world_publish_timestamp = std::chrono::steady_clock::now();

        // Yield to allow other threads to run. This is particularly important if we
        // have this thread and another running on one core
        std::this_thread::yield();

        // TODO: Simulate the primitives
        // https://github.com/UBC-Thunderbots/Software/issues/768
        auto primitives = primitive_buffer.popMostRecentlyAddedValue(primitive_timeout);
        if (!primitives)
        {
            LOG(WARNING) << "Simulator Backend timed out waiting for primitives";
        }

        // Take ownership of the `in_destructor` flag so we can use it for the conditional
        // check
        in_destructor_mutex.lock();
    }
}

// Register this backend in the BackendFactory
static TBackendFactory<SimulatorBackend> factory;
