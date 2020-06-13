#include "software/simulated_tests/simulated_test_fixture.h"

#include "software/logger/logger.h"
#include "software/time/duration.h"

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
