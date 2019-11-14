#include <gtest/gtest.h>

#include "software/backend/simulator_backend.h"
#include "software/gui/visualizer_wrapper.h"
#include "software/test_util/test_util.h"
#include "software/util/logger/init.h"
#include "software/util/time/duration.h"


class SimulatedTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        Util::Logger::LoggerSingleton::initializeLogger();
        backend = std::make_shared<SimulatorBackend>(
            Duration::fromMilliseconds(5), Duration::fromSeconds(1.0 / 30.0),
            SimulatorBackend::SimulationSpeed::FAST_SIMULATION);
    }

    /**
     * This function enables the Visualizer while a test is running, so that the test can
     * be debugged Visually. Simply call this function at the start of the test(s) you
     * want to show in the Visualizer. Make sure to run the test targets with 'bazel run'
     * instead of 'bazel test' or the Visualizer won't work.
     */
    void enableVisualizer()
    {
        // The XDG_RUNTIME_DIR environment variable must be set in order for the
        // Visualizer to work properly. If it's not set, the Visualizer initialization
        // will fail with an error like "qt.qpa.screen: QXcbConnection: Could not connect
        // to display" In order for this environment variable to be set correctly these
        // test targets MUST be run with 'bazel run' rather than 'bazel test'
        auto xdg_runtime_dir = std::getenv("XDG_RUNTIME_DIR");
        if (!xdg_runtime_dir)
        {
            LOG(WARNING)
                << "The XDG_RUNTIME_DIR environment variable was not set. It must"
                   " be set in order for the Visualizer to run properly. If you want"
                   "to enable the Visualizer for these tests, make sure you run the"
                   "targets with 'bazel run' rather than 'bazel test' so the environment"
                   "variables are set properly.";
            ADD_FAILURE() << "Cannot enable Visualizer" << std::endl;
        }

        // We mock empty argc and argv since we don't have access to them when running
        // tests These arguments do not matter for simply running the Visualizer
        char *argv[] = {NULL};
        int argc     = sizeof(argv) / sizeof(char *) - 1;
        visualizer   = std::make_shared<VisualizerWrapper>(argc, argv);
        backend->Subject<World>::registerObserver(visualizer);
        backend->Subject<RobotStatus>::registerObserver(visualizer);

        // Simulate in realtime if we are using the Visualizer so we can actually see
        // things at a reasonably realistic speed
        backend->setSimulationSpeed(
            SimulatorBackend::SimulationSpeed::REALTIME_SIMULATION);
    }

    std::shared_ptr<SimulatorBackend> backend;
    std::shared_ptr<VisualizerWrapper> visualizer;
};

TEST_F(SimulatedTest, example_simulated_test)
{
    World world         = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() = Ball(Point(0, 0), Vector(4, 1.5), Timestamp::fromSeconds(0));

    bool test_succeeded = backend->runSimulation(world, Duration::fromSeconds(10));

    // Currently the simulation always times out because validation is not implemented yet
    ASSERT_FALSE(test_succeeded);
}
