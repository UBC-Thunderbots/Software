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

bool SimulatorBackend::runSimulation(const std::vector<ValidationFunction>& validation_functions, World world, const Duration &timeout)
{
    PhysicsSimulator physics_simulator(world);

    std::shared_ptr<World> world_ptr = std::make_shared<World>(world);

    std::vector<FunctionValidator> function_validators;
    for(const auto& validation_function : validation_functions) {
        function_validators.emplace_back(FunctionValidator(validation_function, world_ptr));
    }

    bool all_tests_pass = runSimulationLoop(world_ptr, function_validators, physics_simulator, timeout);

    if(!all_tests_pass) {
        LOG(WARNING) << "Simulation timed out";
    }

    return all_tests_pass;
}

bool SimulatorBackend::runSimulationLoop(std::shared_ptr<World> world,
                                         std::vector<FunctionValidator> &function_validators,
                                         PhysicsSimulator &physics_simulator, const Duration& timeout) {
    bool all_tests_pass = false;

    std::future<ConstPrimitiveVectorPtr> primitive_future =
            primitive_promise.get_future();
    Subject<World>::sendValueToObservers(*world);
    Timestamp timeout_timestamp = world->getMostRecentTimestamp() + timeout;
    while (world->getMostRecentTimestamp() <= timeout_timestamp)
    {
        all_tests_pass = updateSimulationAndCheckValidation(world, function_validators, physics_simulator);
        if(all_tests_pass) {
            break;
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

        Subject<World>::sendValueToObservers(*world);
    }

    return all_tests_pass;
}

bool SimulatorBackend::updateSimulationAndCheckValidation(std::shared_ptr<World> world,
                                                          std::vector<FunctionValidator> &function_validators,
                                                          PhysicsSimulator &physics_simulator) {
    bool all_tests_pass = false;
    unsigned int num_physics_steps_per_world_published = static_cast<unsigned int>(std::ceil(world_time_increment.getSeconds() /
                                                                                              physics_time_step.getSeconds()));
    for (unsigned int i = 0; i < num_physics_steps_per_world_published; i++)
    {
        *world = physics_simulator.stepSimulation(physics_time_step);

        all_tests_pass = std::all_of(function_validators.begin(), function_validators.end(), [](FunctionValidator& fv){
            return fv.executeAndCheckForSuccess();
        });

        if(all_tests_pass) {
            break;
        }
    }

    return all_tests_pass;
}
