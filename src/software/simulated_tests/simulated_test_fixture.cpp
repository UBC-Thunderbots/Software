#include "software/simulated_tests/simulated_test_fixture.h"

#include "software/logger/logger.h"
#include "software/time/duration.h"
#include "software/test_util/test_util.h"

void SimulatedTest::SetUp()
{
    LoggerSingleton::initializeLogger();
    backend = std::make_shared<SimulatorBackend>(
        Duration::fromMilliseconds(5), Duration::fromSeconds(1.0 / 30.0),
        SimulatorBackend::SimulationSpeed::FAST_SIMULATION);
    world_state_validator = std::make_shared<WorldStateValidator>();
    ai_wrapper =
        std::make_shared<AIWrapper>(Util::DynamicParameters->getAIConfig(),
                                    Util::DynamicParameters->getAIControlConfig());

    // The world_state_observer observes the World from the backend, and then the ai
    // observes the World from the WorldStateObserver. Because we know the AI will not
    // run until it gets a new World, and the SimulatorBackend will not pubish another
    // world until it has received primitives, we can guarantee that each step in the
    // test pipeline will complete before the next. The steps are:
    // 1. Simulate and publish new world
    // 2. Validate World
    // 3. AI makes decisions based on new world
    // Overall this makes the tests deterministic because one step will not asynchronously
    // run way faster than another and lose data.
    backend->Subject<World>::registerObserver(world_state_validator);
    world_state_validator->Subject<World>::registerObserver(ai_wrapper);
    ai_wrapper->Subject<ConstPrimitiveVectorPtr>::registerObserver(backend);
}

void SimulatedTest::TearDown()
{
    backend->stopSimulation();
}

void SimulatedTest::enableVisualizer()
{
    // We mock empty argc and argv since we don't have access to them when running
    // tests These arguments do not matter for simply running the Visualizer
    char *argv[] = {NULL};
    int argc     = sizeof(argv) / sizeof(char *) - 1;
    visualizer   = std::make_shared<VisualizerWrapper>(argc, argv);
    backend->Subject<World>::registerObserver(visualizer);
    backend->Subject<RobotStatus>::registerObserver(visualizer);
    ai_wrapper->Subject<AIDrawFunction>::registerObserver(visualizer);
    ai_wrapper->Subject<PlayInfo>::registerObserver(visualizer);

    // Simulate in realtime if we are using the Visualizer so we can actually see
    // things at a reasonably realistic speed
    backend->setSimulationSpeed(SimulatorBackend::SimulationSpeed::REALTIME_SIMULATION);
}

bool SimulatedTest::validateWorld(std::shared_ptr<World> world_ptr,
                                  std::vector<FunctionValidator> &function_validators,
                                  std::vector<ContinuousFunctionValidator> &continuous_function_validators) {
    for (auto &continuous_function_validator : continuous_function_validators)
    {
        continuous_function_validator.executeAndCheckForFailures();
    }

    bool validation_successful = std::all_of(
            function_validators.begin(), function_validators.end(),
            [](FunctionValidator &fv) { return fv.executeAndCheckForSuccess(); });

    return function_validators.empty() ? false : validation_successful;
}

SimulatedTest::SimulatedTest() : world(nullptr), simulator(::TestUtil::createSSLDivBField()),
    ai(Util::DynamicParameters->getAIConfig(), Util::DynamicParameters->getAIControlConfig())
{
}

void SimulatedTest::runTest(const std::vector<ValidationFunction> &validation_functions,
                            const std::vector<ValidationFunction> &continuous_validation_functions,
                            const Duration &timeout) {
    // Setup function validators
    for (ValidationFunction validation_function : validation_functions)
    {
        function_validators.emplace_back(
                FunctionValidator(validation_function, world));
    }

    for (const auto &continuous_validation_function : continuous_validation_functions)
    {
        continuous_function_validators.emplace_back(
                ContinuousFunctionValidator(continuous_validation_function, world));
    }

    Timestamp current_time = Timestamp::fromSeconds(0.0);
    Timestamp timeout_time = current_time + timeout;
    Duration dt = Duration::fromSeconds(1.0 / 60.0);
    while(current_time < timeout_time) {
        simulator.stepSimulation(dt);
        auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
        assert(ssl_wrapper_packet);

        auto sensor_msg = SensorMsg();
        sensor_msg.set_allocated_ssl_vision_msg(ssl_wrapper_packet.release());

        sensor_fusion.updateWorld(sensor_msg);
        std::optional<World> world_opt = sensor_fusion.getWorld();

        if(!world_opt) {
            // TODO: log
            continue;
        }

        // TODO: You are here. Just got super hacky skeleton of new synchronous test fixture working
        // * need to check if the world member variable gets updated without its address changing
        //   so that validation functions work

        world = std::make_shared<World>(world_opt.value());
//        *world = world_opt.value();

        bool stop_test = SimulatedTest::validateWorld(world, function_validators, continuous_function_validators);
        if(stop_test) {
            // TODO: log
            break;
        }

        auto p = ai.getPrimitives(*world);
        auto new_primitives_ptr =
                std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
                        std::move(p));

        simulator.setYellowRobotPrimitives(new_primitives_ptr);

        current_time = current_time + dt;
    }

//    if (!validation_successful && !function_validators.empty())
//    {
//        LOG(WARNING)
//            << "Validation failed. Not all validation functions passed within the timeout duration";
//        return false;
//    }
//    else
//    {
//        return true;
//    }
}
