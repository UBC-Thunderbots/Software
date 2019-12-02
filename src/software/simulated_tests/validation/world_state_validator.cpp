#include "software/simulated_tests/validation/world_state_validator.h"

#include "software/simulated_tests/validation/continuous_function_validator.h"
#include "software/simulated_tests/validation/function_validator.h"
#include "software/util/logger/init.h"

WorldStateValidator::WorldStateValidator() : world_buffer(world_buffer_size) {}

void WorldStateValidator::onValueReceived(World world)
{
    world_buffer.push(world);
}

bool WorldStateValidator::waitForValidationToPass(
    const std::vector<ValidationFunction> &validation_functions,
    const std::vector<ValidationFunction> &continuous_validation_functions,
    const Duration &timeout)
{
    std::optional<World> world =
        world_buffer.popLeastRecentlyAddedValue(world_buffer_timeout);
    if (!world)
    {
        LOG(WARNING)
            << "WorldStateValidator timed out waiting for the initial World to be received";
        return false;
    }

    // The pointer to the world that will be shared with all the function validators
    std::shared_ptr<World> world_ptr = std::make_shared<World>(world.value());

    // THIS WORKS
//    std::vector<FunctionValidator> function_validators;
//    FunctionValidator foo(validation_functions.at(0), world_ptr);
//    std::cout << "emplacing function validator foo: " << &foo << std::endl;
//    function_validators.emplace_back(std::move(foo));
//    FunctionValidator bar(validation_functions.at(1), world_ptr);
//    std::cout << "function validator bar: " << &bar << std::endl;
//    function_validators.emplace_back(std::move(bar));

    // THIS DOES NOT WORK
//    std::vector<FunctionValidator> function_validators;
//    for(unsigned int i = 0; i < validation_functions.size(); i++) {
//        FunctionValidator foo(validation_functions.at(i), world_ptr);
//        std::cout << "emplacing function validator: " << &foo << std::endl;
//        function_validators.emplace_back(std::move(foo));
//    }

    // THIS DOES NOT WORK
    std::vector<FunctionValidator> function_validators;
    for (ValidationFunction validation_function : validation_functions)
    {
        FunctionValidator foo(validation_function, world_ptr);
        std::cout << "emplacing function validator: " << &foo << std::endl;
        function_validators.emplace_back(std::move(foo));
    }

    for(const auto& function_validator : function_validators) {
        std::cout << "validator after emplace " << &function_validator << std::endl;
    }

    for(unsigned int i = 0; i < validation_functions.size(); i++) {
        function_validators[i].setValidationFunction(validation_functions[i]);
    }


    std::vector<ContinuousFunctionValidator> continuous_function_validators;
    for (const auto &continuous_validation_function : continuous_validation_functions)
    {
        continuous_function_validators.emplace_back(
            ContinuousFunctionValidator(continuous_validation_function, world_ptr));
    }

    bool validation_successful   = false;
    Timestamp starting_timestamp = world_ptr->getMostRecentTimestamp();
    Timestamp timeout_timestamp  = starting_timestamp + timeout;
    while (world_ptr->getMostRecentTimestamp() < timeout_timestamp)
    {
        for (auto &continuous_function_validator : continuous_function_validators)
        {
            continuous_function_validator.executeAndCheckForFailures();
        }

        validation_successful = std::all_of(
            function_validators.begin(), function_validators.end(),
            [](FunctionValidator &fv) { return fv.executeAndCheckForSuccess(); });
        if (validation_successful && !function_validators.empty())
        {
            break;
        }

        // After we have validated the world state, send it to other Observers
        Subject<World>::sendValueToObservers(*world_ptr);

        std::optional<World> world =
            world_buffer.popLeastRecentlyAddedValue(world_buffer_timeout);
        if (!world)
        {
            LOG(WARNING)
                << "WorldStateValidator timed out waiting for the initial World to be received";
            return false;
        }
        // We update the value of the existing pointer rather than making a new pointer
        // because we have shared this pointer with the function validators, and need
        // the pointer to stay the same for values to be shared properly
        *world_ptr = world.value();
    }

    if (!validation_successful && !function_validators.empty())
    {
        LOG(INFO)
            << "Validation failed. Not all validation functions passed within the timeout duration";
    }
    else
    {
        LOG(INFO) << "Validation passed!";
    }

    return validation_successful;
}
