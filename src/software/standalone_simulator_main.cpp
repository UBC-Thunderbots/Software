#include <boost/program_options.hpp>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/gui/standalone_simulator/threaded_standalone_simulator_gui.h"
#include "software/logger/logger.h"
#include "software/simulation/standalone_simulator.h"

int main(int argc, char **argv)
{
    // load command line arguments
    auto args           = std::make_shared<StandaloneSimulatorMainCommandLineArgs>();
    bool help_requested = args->loadFromCommandLineArguments(argc, argv);

    LoggerSingleton::initializeLogger(args->getLoggingDir()->value());

    if (!help_requested)
    {
        // Setup dynamic parameters
        auto mutable_thunderbots_config = std::make_shared<ThunderbotsConfig>();
        auto thunderbots_config =
            std::const_pointer_cast<const ThunderbotsConfig>(mutable_thunderbots_config);

        // Override default network interface
        if (!args->getInterface()->value().empty())
        {
            mutable_thunderbots_config->getMutableStandaloneSimulatorConfig()
                ->getMutableNetworkInterface()
                ->setValue(args->getInterface()->value());
        }

        // Experimentally determined restitution value
        mutable_thunderbots_config->getMutableSimulatorConfig()
            ->getMutableBallRestitution()
            ->setValue(0.8);
        // Measured these values from fig. 9 on page 8 of
        // https://ssl.robocup.org/wp-content/uploads/2020/03/2020_ETDP_ZJUNlict.pdf
        mutable_thunderbots_config->getMutableSimulatorConfig()
            ->getMutableSlidingFrictionAcceleration()
            ->setValue(6.9);
        mutable_thunderbots_config->getMutableSimulatorConfig()
            ->getMutableRollingFrictionAcceleration()
            ->setValue(0.5);

        std::shared_ptr<StandaloneSimulator> standalone_simulator;

        // Setup the field
        if (args->getSslDivision()->value() == "div_a")
        {
            standalone_simulator = std::make_shared<StandaloneSimulator>(
                mutable_thunderbots_config->getMutableStandaloneSimulatorConfig(),
                mutable_thunderbots_config->getMutableSimulatorConfig(),
                Field::createSSLDivisionAField());

            standalone_simulator->setupInitialSimulationState(DIV_A_NUM_ROBOTS);
        }
        else
        {
            standalone_simulator = std::make_shared<StandaloneSimulator>(
                mutable_thunderbots_config->getMutableStandaloneSimulatorConfig(),
                mutable_thunderbots_config->getMutableSimulatorConfig(),
                Field::createSSLDivisionBField());

            standalone_simulator->setupInitialSimulationState(DIV_B_NUM_ROBOTS);
        }

        ThreadedStandaloneSimulatorGUI threaded_standalone_simulator_gui(
            standalone_simulator, mutable_thunderbots_config->getMutableSimulatorConfig(),
            mutable_thunderbots_config->getMutableStandaloneSimulatorConfig());

        // This blocks forever without using the CPU.
        // Wait for the Simulator GUI to shut down before shutting
        // down the rest of the system
        threaded_standalone_simulator_gui.getTerminationPromise()->get_future().wait();
    }

    return 0;
}
