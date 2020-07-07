#include <boost/program_options.hpp>

#include "software/gui/standalone_simulator/threaded_standalone_simulator_gui.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/simulation/standalone_simulator.h"

struct commandLineArgs
{
    bool help                          = false;
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

    std::string interface_help_str =
        "The interface to send and receive packets over (can be found through ifconfig)";

    boost::program_options::options_description desc{"Options"};
    desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                       "Help screen");
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
            MutableDynamicParameters->getMutableStandaloneSimulatorConfig()
                ->mutableNetworkInterface()
                ->setValue(args.network_interface_name);
        }

        ThreadedStandaloneSimulatorGUI standalone_simulator_gui_wrapper;
        StandaloneSimulator standalone_simulator(
            MutableDynamicParameters->getMutableStandaloneSimulatorConfig());
        standalone_simulator.registerOnSSLWrapperPacketReadyCallback(
            [&standalone_simulator_gui_wrapper](SSL_WrapperPacket packet) {
                standalone_simulator_gui_wrapper.onValueReceived(packet);
            });

        // This blocks forever without using the CPU.
        // Wait for the Simulator GUI to shut down before shutting
        // down the rest of the system
        standalone_simulator_gui_wrapper.getTerminationPromise()->get_future().wait();
    }

    return 0;
}
