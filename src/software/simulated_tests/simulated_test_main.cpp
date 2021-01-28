#include <gtest/gtest.h>

#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/simulated_tests/simulated_test_fixture.h"

bool SimulatedTestFixture::enable_visualizer = false;

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    // load command line arguments
    auto args = MutableDynamicParameters->getMutableSimulatedTestMainCommandLineArgs();
    bool help_requested = args->loadFromCommandLineArguments(argc, argv);

    LoggerSingleton::initializeLogger(args->LoggingDir()->value());

    if (!help_requested)
    {
        SimulatedTestFixture::enable_visualizer = args->EnableVisualizer()->value();
    }

    return RUN_ALL_TESTS();
}
