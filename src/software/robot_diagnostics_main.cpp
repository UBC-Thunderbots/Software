#include <boost/program_options.hpp>
#include <iostream>
#include <numeric>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/backend/backend.h"
#include "software/gui/robot_diagnostics/threaded_robot_diagnostics_gui.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

int main(int argc, char **argv)
{
    // load command line arguments
    auto args           = std::make_shared<RobotDiagnosticsMainCommandLineArgs>();
    bool help_requested = args->loadFromCommandLineArguments(argc, argv);

    LoggerSingleton::initializeLogger(args->getRuntimeDir()->value());

    if (!help_requested)
    {
        // Setup dynamic parameters
        auto mutable_thunderbots_config = std::make_shared<ThunderbotsConfig>();
        auto thunderbots_config =
            std::const_pointer_cast<const ThunderbotsConfig>(mutable_thunderbots_config);

        // Override default network interface
        if (!args->getInterface()->value().empty())
        {
            mutable_thunderbots_config->getMutableNetworkConfig()
                ->getMutableNetworkInterface()
                ->setValue(args->getInterface()->value());
        }

        CHECK(!args->getBackend()->value().empty())
            << "The option '--backend' is required but missing";

        std::shared_ptr<Backend> backend =
            GenericFactory<std::string, Backend, BackendConfig>::create(
                args->getBackend()->value(), thunderbots_config->getBackendConfig());

        std::shared_ptr<ThreadedRobotDiagnosticsGUI> threaded_robot_diagnostics_gui;
        threaded_robot_diagnostics_gui =
            std::make_shared<ThreadedRobotDiagnosticsGUI>(argc, argv);
        threaded_robot_diagnostics_gui->registerObserver(backend);
        backend->Subject<SensorProto>::registerObserver(threaded_robot_diagnostics_gui);
        // This blocks forever without using the CPU
        // Wait for the GUI to shut down before shutting
        // down the rest of the system
        threaded_robot_diagnostics_gui->getTerminationPromise()->get_future().wait();
    }

    return 0;
}
