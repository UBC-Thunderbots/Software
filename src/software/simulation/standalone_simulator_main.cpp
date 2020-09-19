#include <boost/program_options.hpp>

#include "software/gui/standalone_simulator/threaded_standalone_simulator_gui.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/simulation/standalone_simulator.h"

int main(int argc, char **argv)
{
    LoggerSingleton::initializeLogger();

    // load command line arguments
    auto args =
        MutableDynamicParameters->getMutableStandaloneSimulatorMainCommandLineArgs();
    bool help_requested = args->loadFromCommandLineArguments(argc, argv);

    if (!help_requested)
    {
        // TODO remove this when we move to the new dynamic parameter system
        // https://github.com/UBC-Thunderbots/Software/issues/1298
        if (!args->interface()->value().empty())
        {
            MutableDynamicParameters->getMutableStandaloneSimulatorConfig()
                ->mutableNetworkInterface()
                ->setValue(args->interface()->value());
        }

        auto standalone_simulator = std::make_shared<StandaloneSimulator>(
            MutableDynamicParameters->getMutableStandaloneSimulatorConfig());
        standalone_simulator->setupInitialSimulationState();

        ThreadedStandaloneSimulatorGUI threaded_standalone_simulator_gui(
            standalone_simulator);

        // This blocks forever without using the CPU.
        // Wait for the Simulator GUI to shut down before shutting
        // down the rest of the system
        threaded_standalone_simulator_gui.getTerminationPromise()->get_future().wait();
    }

    return 0;
}
