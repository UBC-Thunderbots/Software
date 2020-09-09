#include <boost/program_options.hpp>
#include <iostream>
#include <numeric>

#include "software/backend/backend.h"
#include "software/gui/robot_diagnostics/threaded_robot_diagnostics_gui.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"

struct commandLineArgs
{
    bool help                          = false;
    std::string backend_name           = "";
    std::string network_interface_name = "";
    bool err                           = false;
};

/**
 * Parses arguments and indicates which arguments were received
 *
 * @param argc
 * @param argv
 *
 * @return a struct of which arguments are passed
 */
commandLineArgs parseCommandLineArgs(int argc, char **argv)
{
    commandLineArgs args;
    // Build one string with all the backend_names
    std::vector<std::string> backend_names =
        GenericFactory<std::string, Backend>::getRegisteredNames();

    std::string all_backend_names =
        std::accumulate(std::begin(backend_names), std::end(backend_names), std::string(),
                        [](std::string &ss, std::string &s) { return ss + s + ", "; });
    std::string backend_help_str =
        "The backend that you would like to use, one of: " + all_backend_names;

    std::string interface_help_str =
        "The interface to send and receive packets over (can be found through ifconfig)";

    boost::program_options::options_description desc{"Options"};
    desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                       "Help screen");
    desc.add_options()("backend",
                       boost::program_options::value<std::string>(&args.backend_name),
                       backend_help_str.c_str());
    desc.add_options()(
        "interface",
        boost::program_options::value<std::string>(&args.network_interface_name),
        interface_help_str.c_str());

    boost::program_options::variables_map vm;
    try
    {
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        if (args.help)
        {
            std::cout << desc << std::endl;
        }
    }
    catch (const boost::program_options::error &ex)
    {
        std::cerr << ex.what() << '\n';
        args.err = true;
        return args;
    }

    return args;
}

int main(int argc, char **argv)
{
    LoggerSingleton::initializeLogger();

    commandLineArgs args = parseCommandLineArgs(argc, argv);

    if (!args.help && !args.err)
    {
        // TODO remove this when we move to non-generic factories for backends
        // https://github.com/UBC-Thunderbots/Software/issues/1452
        if (!args.network_interface_name.empty())
        {
            MutableDynamicParameters->getMutableNetworkConfig()
                ->mutableNetworkInterface()
                ->setValue(args.network_interface_name);
        }

        if (args.backend_name.empty())
        {
            LOG(FATAL) << "The option '--backend' is required but missing";
        }

        std::shared_ptr<Backend> backend =
            GenericFactory<std::string, Backend>::create(args.backend_name);

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
