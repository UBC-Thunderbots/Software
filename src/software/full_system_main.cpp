#include <boost/program_options.hpp>
#include <iostream>
#include <numeric>

#include "software/ai/ai_wrapper.h"
#include "software/ai/hl/stp/play_info.h"
#include "software/backend/backend.h"
#include "software/constants.h"
#include "software/gui/visualizer/visualizer_wrapper.h"
#include "software/logger/logger.h"
#include "software/util/design_patterns/generic_factory.h"


using namespace boost::program_options;
// Member variables we need to maintain state
// They are kept in an anonymous namespace so they are not accessible outside this
// file and are not created as global static variables.
namespace
{
    std::shared_ptr<AIWrapper> ai;
    std::shared_ptr<Backend> backend;
    std::shared_ptr<VisualizerWrapper> visualizer;
}  // namespace

struct commandLineArgs
{
    bool help                = false;
    std::string backend_name = "";
    bool headless            = false;
    bool err                 = false;
};

// clang-format off
std::string BANNER =
"        ,/                                                                                                                     ,/   \n"
"      ,'/         _    _ ____   _____   _______ _    _ _    _ _   _ _____  ______ _____  ____   ____ _______ _____           ,'/    \n"
"    ,' /         | |  | |  _ \\ / ____| |__   __| |  | | |  | | \\ | |  __ \\|  ____|  __ \\|  _ \\ / __ \\__   __/ ____|        ,' /     \n"
"  ,'  /_____,    | |  | | |_) | |         | |  | |__| | |  | |  \\| | |  | | |__  | |__) | |_) | |  | | | | | (___        ,'  /_____,\n"
".'____    ,'     | |  | |  _ <+ |         | |  |  __  | |  | | . ` | |  | |  __| |  _  /|  _ <+ |  | | | |  \\___ \\     .'____    ,' \n"
"     /  ,'       | |__| | |_) | |____     | |  | |  | | |__| | |\\  | |__| | |____| | \\ \\| |_) | |__| | | |  ____) |         /  ,'   \n"
"    / ,'          \\____/|____/ \\_____|    |_|  |_|  |_|\\____/|_| \\_|_____/|______|_|  \\_\\____/ \\____/  |_| |_____/         / ,'     \n"
"   /,'                                                                                                                    /,'       \n"
"  /'                                                                                                                     /'          \n";
// clang-format on

void setBackendFromString(std::string backend_name)
{
    try
    {
        backend = GenericFactory<std::string, Backend>::create(backend_name);
    }
    catch (const std::invalid_argument &e)
    {
        LOG(FATAL) << e.what();
    }
}

/**
 * Parses Arguments and Indicates which arguments were received
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

    options_description desc{"Options"};
    desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                       "Help screen")("backend",
                                      value<std::string>(&args.backend_name)->required(),
                                      backend_help_str.c_str())(
        "headless", boost::program_options::bool_switch(&args.headless),
        "Run without the Visualizer");

    variables_map vm;
    try
    {
        store(parse_command_line(argc, argv, desc), vm);
        notify(vm);

        if (args.help)
        {
            std::cout << desc << std::endl;
        }
    }
    catch (const error &ex)
    {
        std::cerr << ex.what() << '\n';
        args.err = true;
        return args;
    }

    return args;
}

/**
 * Connects all the observers together
 *
 * @param headless - determines whether or not the visualizer is run (false to run
 * visualizer)
 */
void connectObservers(bool headless)
{
    backend->Subject<World>::registerObserver(ai);
    ai->Subject<ConstPrimitiveVectorPtr>::registerObserver(backend);
    if (!headless)
    {
        backend->Subject<World>::registerObserver(visualizer);
        ai->Subject<AIDrawFunction>::registerObserver(visualizer);
        ai->Subject<PlayInfo>::registerObserver(visualizer);
        backend->Subject<RobotStatus>::registerObserver(visualizer);
    }
}

int main(int argc, char **argv)
{
    std::cout << BANNER << std::endl;

    LoggerSingleton::initializeLogger();

    commandLineArgs args = parseCommandLineArgs(argc, argv);

    if (!args.help && !args.err)
    {
        // Setup dynamic parameters
        // TODO (Issue #960): Once we're using injected parameters everywhere (instead of
        //                    just global accesses, `Util::DynamicParameters` should be
        //                    deleted, and we should just create an instance here instead)
        std::shared_ptr<const AIConfig> ai_config =
            Util::DynamicParameters->getAIConfig();
        std::shared_ptr<const AIControlConfig> ai_control_config =
            Util::DynamicParameters->getAIControlConfig();

        // The ai has to be initialized after the backend (which is started in
        // parseCommandLineArgs) This is a bug. See #834
        ai = std::make_shared<AIWrapper>(ai_config, ai_control_config);

        setBackendFromString(args.backend_name);

        if (!args.headless)
        {
            visualizer = std::make_shared<VisualizerWrapper>(argc, argv);
        }

        connectObservers(args.headless);

        if (!args.headless)
        {
            // This blocks forever without using the CPU
            // Wait for the visualizer to shut down before shutting
            // down the rest of the system
            visualizer->getTerminationPromise()->get_future().wait();
        }
        else
        {
            // This blocks forever without using the CPU
            std::promise<void>().get_future().wait();
        }
    }

    return 0;
}
