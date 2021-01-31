#include <boost/program_options.hpp>
#include <iostream>
#include <numeric>

#include "software/backend/backend.h"
#include "software/gui/robot_diagnostics/threaded_robot_diagnostics_gui.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"

int main(int argc, char **argv)
{
    // load command line arguments
    auto args = MutableDynamicParameters->getMutableRobotDiagnosticsMainCommandLineArgs();
    bool help_requested = args->loadFromCommandLineArguments(argc, argv);

    LoggerSingleton::initializeLogger(args->getLoggingDir()->value());

    if (!help_requested)
    {
        // TODO remove this when we move to the new dynamic parameter system
        // https://github.com/UBC-Thunderbots/Software/issues/1298
        if (!args->getInterface()->value().empty())
        {
            MutableDynamicParameters->getMutableNetworkConfig()
                ->getMutableNetworkInterface()
                ->setValue(args->getInterface()->value());
        }

        if (args->getBackend()->value().empty())
        {
            LOG(FATAL) << "The option '--backend' is required but missing";
        }

        std::shared_ptr<Backend> backend =
            GenericFactory<std::string, Backend>::create(args->getBackend()->value());

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
