#include <boost/program_options.hpp>
#include <g3log/g3log.hpp>
#include <iostream>
#include <numeric>

#include "software/ai/ai_wrapper.h"
#include "software/ai/hl/stp/play_info.h"
#include "software/backend/backend_factory.h"
#include "software/gui/drawing/draw_functions.h"
#include "software/gui/visualizer_wrapper.h"
#include "software/typedefs.h"
#include "software/util/canvas_messenger/canvas_messenger.h"
#include "software/util/constants.h"
#include "software/util/logger/init.h"

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
    BackendFactory backend_factory;
    try
    {
        backend = backend_factory.createBackend(backend_name);
    }
    catch (const std::invalid_argument &e)
    {
        LOG(FATAL) << e.what();
    }
}

/**
 * Parses Arguments and Indicates If The Program Should Continue
 *
 * @param argc
 * @param argv
 *
 * @return True if the program should continue, false otherwise
 */
bool parseCommandLineArgs(int argc, char **argv)
{
    // Build one string with all the backend_names
    std::vector<std::string> backend_names = BackendFactory().getRegisteredBackendNames();
    std::string all_backend_names =
        std::accumulate(std::begin(backend_names), std::end(backend_names), std::string(),
                        [](std::string &ss, std::string &s) { return ss + s + ", "; });
    std::string backend_help_str =
        "The backend that you would like to use, one of: " + all_backend_names;

    try
    {
        options_description desc{"Options"};
        desc.add_options()("help,h", "Help screen")(
            "backend", value<std::string>()->notifier(setBackendFromString)->required(),
            backend_help_str.c_str());

        variables_map vm;
        store(parse_command_line(argc, argv, desc), vm);

        // We only process notifications if "help" was not given, which allows us to
        // avoid issues where required arguments are not required if "help" is given
        if (vm.count("help"))
        {
            std::cout << BANNER << std::endl << desc << std::endl;
            return false;
        }
        else
        {
            notify(vm);
        }
    }
    catch (const error &ex)
    {
        std::cerr << ex.what() << '\n';
    }

    return true;
}

/**
 * Connects all the observers together
 */
void connectObservers()
{
    backend->Subject<World>::registerObserver(ai);
    backend->Subject<World>::registerObserver(visualizer);
    backend->Subject<RobotStatus>::registerObserver(visualizer);
    ai->Subject<ConstPrimitiveVectorPtr>::registerObserver(backend);
    ai->Subject<AIDrawFunction>::registerObserver(visualizer);
    ai->Subject<PlayInfo>::registerObserver(visualizer);
}

int main(int argc, char **argv)
{
    Util::Logger::LoggerSingleton::initializeLogger();

    visualizer = std::make_shared<VisualizerWrapper>(argc, argv);
    ai         = std::make_shared<AIWrapper>();

    if (parseCommandLineArgs(argc, argv))
    {
        connectObservers();

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }

    return 0;
}
