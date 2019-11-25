#pragma once

#include <future>
#include <mutex>

#include "software/backend/backend.h"
#include "software/backend/simulation/physics/physics_simulator.h"
#include "software/backend/simulation/validation/validation_function.h"
#include "software/backend/simulation/validation/function_validator.h"
#include "software/world/world.h"

/**
 * This class implements the Backend interface using a physics simulation to
 * execute received commands and provide new data to the system.
 *
 * This class is intentionally not registered in the BackendFactory since it does
 * not have a default constructor and has a relatively specialized use case.
 */
class SimulatorBackend : public Backend
{
   public:
    static const std::string name;

    /**
     * Supported modes for simulation speed.
     *
     * - FAST_SIMULATION will run the simulation as fast a possible
     * - REALTIME_SIMULATION will run the simulation in real-time, meaning
     *   if timestamps in published data are 'n' seconds apart, they will be published
     *   'n' seconds apart in real "wall-clock" time. This is useful
     *   if you want to visualize the simulation.
     */
    enum SimulationSpeed
    {
        FAST_SIMULATION,
        REALTIME_SIMULATION
    };

    /**
     * Creates a new SimulatorBackend
     *
     * The SimulatorBackend will run a physics simulation to simulate and publish the
     * World, and simulate the primitives given to it. The SimulatorBackend is
     * deterministic and so will advance the simulation by a fixed amount each time
     * primitives are received, rather than running completely independently at full-speed
     * or in real-time.
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
     * the AI is ready for more data. This is the scenario we are trying to avoid by
     * waiting for Primitives before continuing the simulation.
     *
     * @param physics_time_step Time time step used by the physics simulation
     * @param world_time_increment The increment in the timestamp of the published World
     * object each time it is published. ie. from the perspective of consumers, the World
     * is being published every 'world_time_increment' seconds.
     * @param simulation_speed_mode What speed mode to run the simulator with
     */
    explicit SimulatorBackend(const Duration& physics_time_step,
                              const Duration& world_time_increment,
                              SimulationSpeed simulation_speed_mode);

    /**
     * Sets the SimulationSpeed for the simulation
     *
     * @param simulation_speed_mode the new SimulationSpeed to set
     */
    void setSimulationSpeed(SimulationSpeed simulation_speed_mode);

    /**
     * Simulates the world given the initial state until the simulation times out
     *
     * @param world The initial state of the world in the simulation
     * @param timeout How long to run the simulation for before failing
     * @return true if the simulation succeeds, and false if it times out and fails
     */
    bool runSimulation(const std::vector<ValidationFunction>& validation_functions, World world, const Duration& timeout);

   private:
    /**
     * Runs the main simulation loop that updates the physics simulation, checks the
     * function validators, and publishes the latest world data to Observers
     *
     * @param world The world to be simulated
     * @param function_validators The function validators to check during the simulation
     * @param physics_simulator The physics simulator responsible for the physics simulation
     * @param timeout How long to run the simulation before timing out
     *
     * @return true if all function_validators report their validation function has passed
     * before the simulation timeout is reached, and false otherwise
     */
    bool runSimulationLoop(std::shared_ptr<World> world,
                           std::vector<FunctionValidator>& function_validators,
                           PhysicsSimulator& physics_simulator,
                           const Duration& timeout);

    /**
     * Updates the simulation until world_time_increment has passed. For each physics step
     * checks the result of each function_validator. If all function_validators report their
     * validation_functions have passed, this function returns true.
     *
     * @param world The world to be simulated
     * @param function_validators The function validators to check during the simulation
     * @param physics_simulator The physics simulator responsible for the physics simulation
     *
     * @return true if all function_validators report their validation function has passed
     * before the simulation timeout is reached, and false otherwise
     */
    bool updateSimulationAndCheckValidation(std::shared_ptr<World> world,
            std::vector<FunctionValidator>& function_validators,
            PhysicsSimulator& physics_simulator);

    void onValueReceived(ConstPrimitiveVectorPtr primitives) override;

    // The time increment the physics simulation is updated by
    const Duration physics_time_step;
    // The time increment between each World published by the backend
    const Duration world_time_increment;
    SimulationSpeed simulation_speed_mode;

    std::promise<ConstPrimitiveVectorPtr> primitive_promise;
};
