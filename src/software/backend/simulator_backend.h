#pragma once

#include <mutex>

#include "software/backend/backend.h"
#include "software/world/world.h"

class SimulatorBackend : public Backend
{
   public:
    static const std::string name;

    /**
     * Creates a new SimulatorBackend
     *
     * The SimulatorBackend will run a physics simulation to simulate and publish the
     * World, and simulate the primitives given to it. The SimulatorBackend will only
     * advance the simulation by a fixed amount each time primitives are received, rather
     * than running at full-speed or in real-time. This is because we want the simulator
     * to be deterministic and always provide the World at a "fixed" rate from the
     * perspective of any consumers.
     *
     * The reason we don't want to run in real-time is because the simulation can likely
     * run much faster than real life. This is a problem because if the simulation can run
     * at 300Hz (for example), but the AI can only run at 30Hz, the buffers between the
     * Observers will overflow and data will be lost. This is not realistic because
     * generally the input runs at 60Hz (roughly the same rate as the camera fps) and the
     * AI runs at about 30Hz.
     *
     * This means that in real life each time the AI is ready for more data, about 2 *
     * 1/60 seconds have passed in real-time (and therefore World time). But if the
     * simulation is running way faster then 10 * 1/60 seconds will have passed each time
     * the AI is ready for more data. This is the scenario we are trying to avoid.
     *
     * @param simulation_time_step How far to step the simulation in time each time the
     * simulation is advanced
     * @param num_steps_per_primitive_update How many times to advance the simulation each
     * time new primitives are received
     */
    explicit SimulatorBackend(
        const Duration& simulation_time_step        = Duration::fromSeconds(1.0 / 60.0),
        unsigned int num_steps_per_primitive_update = 2);

    /**
     * Returns the current state of the World in the simulated backend
     *
     * @return the current state of the World in the simulated backend
     */
    World getWorld();

    /**
     * Sets the state of the World in the simulated backend. All following simulation will
     * follow from this newly provided world.
     *
     * @param new_world A new state of the World to set in the simulated backend
     */
    void setWorld(const World& new_world);

   private:
    /**
     * Updates the simulation.
     *
     * This will advance the simulation by "simulation_time_step",
     * "num_steps_per_primitive_update" times. We take several smaller steps if necessary
     * to maintain the accuracy of the simulation. This will update the state of the World
     * in the backend and send the updated values to all observers.
     */
    void updateSimulation();

    /**
     * Sends the current state of the world in the simulation to all Observers
     */
    void sendWorldToObservers();

    void onValueReceived(ConstPrimitiveVectorPtr primitives) override;

    Duration simulation_time_step;
    unsigned int num_steps_per_primitive_update;

    ConstPrimitiveVectorPtr most_recently_received_primitives;
    std::mutex most_recently_received_primitives_mutex;

    // The current state of the world in the simulation
    World world;
    std::mutex world_mutex;
};
