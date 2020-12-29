#include <gtest/gtest.h>

#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/simulated_tests/simulated_action_tests/simulated_action_test_fixture.h"

bool SimulatedActionTestFixture::enable_visualizer = false;

int main(int argc, char **argv)
{
    LoggerSingleton::initializeLogger();

    testing::InitGoogleTest(&argc, argv);

    // load command line arguments
    auto args = MutableDynamicParameters->getMutableSimulatedTestMainCommandLineArgs();
    bool help_requested = args->loadFromCommandLineArguments(argc, argv);

    if (!help_requested)
    {
        SimulatedActionTestFixture::enable_visualizer =
            args->enable_visualizer()->value();
    }

    return RUN_ALL_TESTS();
}
