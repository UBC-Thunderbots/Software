#include <gtest/gtest.h>

#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/simulated_tests/simulated_test_fixture.h"

bool SimulatedTestFixture::enable_visualizer = false;
bool SimulatedTestFixture::stop_ai_on_start  = false;

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    // load command line arguments
    auto args = MutableDynamicParameters->getMutableSimulatedTestMainCommandLineArgs();
    bool help_requested = args->loadFromCommandLineArguments(argc, argv);

    LoggerSingleton::initializeLogger(args->getLoggingDir()->value());

    if (!help_requested)
    {
        SimulatedTestFixture::enable_visualizer = args->getEnableVisualizer()->value();
        if (SimulatedTestFixture::enable_visualizer)
        {
            SimulatedTestFixture::stop_ai_on_start = args->getStopAiOnStart()->value();
        }
    }

    return RUN_ALL_TESTS();
}
