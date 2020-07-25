#include <boost/program_options.hpp>
#include <experimental/filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>

#include "software/ai/ai_wrapper.h"
#include "software/ai/hl/stp/play_info.h"
#include "software/backend/backend.h"
#include "software/backend/replay_logging/replay_logger.h"
#include "software/constants.h"
#include "software/gui/full_system/threaded_full_system_gui.h"
#include "software/logger/logger.h"
#include "software/sensor_fusion/threaded_sensor_fusion.h"
#include "software/util/design_patterns/generic_factory.h"

struct commandLineArgs
{
    bool help                          = false;
    std::string backend_name           = "";
    std::string network_interface_name = "";
    std::string replay_path            = "";
    bool headless                      = false;
    bool err                           = false;
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

namespace fs = std::experimental::filesystem;

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

    std::string replay_dir_help_str = "The sensor msg log directory to playback";

    boost::program_options::options_description desc{"Options"};
    desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                       "Help screen");
    desc.add_options()(
        "backend",
        boost::program_options::value<std::string>(&args.backend_name)->required(),
        backend_help_str.c_str());
    desc.add_options()(
        "interface",
        boost::program_options::value<std::string>(&args.network_interface_name),
        interface_help_str.c_str());
    desc.add_options()("replay_path",
                       boost::program_options::value<std::string>(&args.replay_path),
                       replay_dir_help_str.c_str());
    desc.add_options()("headless", boost::program_options::bool_switch(&args.headless),
                       "Run without the FullSystemGUI");

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
    std::cout << BANNER << std::endl;

    LoggerSingleton::initializeLogger();

    commandLineArgs args = parseCommandLineArgs(argc, argv);

    if (!args.help && !args.err)
    {
        // Setup dynamic parameters
        // TODO (Issue #960): Once we're using injected parameters everywhere (instead of
        //                    just global accesses, `DynamicParameters` should be
        //                    deleted, and we should just create an instance here instead)
        std::shared_ptr<const AIConfig> ai_config = DynamicParameters->getAIConfig();
        std::shared_ptr<const AIControlConfig> ai_control_config =
            DynamicParameters->getAIControlConfig();
        std::shared_ptr<const SensorFusionConfig> sensor_fusion_config =
            DynamicParameters->getSensorFusionConfig();
        std::shared_ptr<const ReplayLoggingConfig> replay_logging_config =
            DynamicParameters->getReplayLoggingConfig();
        std::shared_ptr<const ReplayBackendConfig> replay_backend_config =
            DynamicParameters->getReplayBackendConfig();

        // TODO remove this when we move to non-generic factories for backends
        // https://github.com/UBC-Thunderbots/Software/issues/1452
        if (!args.network_interface_name.empty())
        {
            MutableDynamicParameters->getMutableNetworkConfig()
                ->mutableNetworkInterface()
                ->setValue(args.network_interface_name);
        }

        // if the replay backend is selected, check that the replay directory is set
        // and set the parameter from the command line option before the ReplayBackend
        // is constructed
        if (args.backend_name == "replay")
        {
            if (args.replay_path.empty())
            {
                throw std::invalid_argument(
                    "No replay directory given! Please give a "
                    "replay directory to playback with the "
                    "--replay_path parameter");
            }

            MutableDynamicParameters->getMutableReplayBackendConfig()
                ->mutableReplayPath()
                ->setValue(args.replay_path);
        }

        std::shared_ptr<Backend> backend =
            GenericFactory<std::string, Backend>::create(args.backend_name);
        auto sensor_fusion = std::make_shared<ThreadedSensorFusion>(sensor_fusion_config);
        auto ai            = std::make_shared<AIWrapper>(ai_config, ai_control_config);
        std::shared_ptr<ThreadedFullSystemGUI> visualizer;

        // Connect observers
        ai->Subject<ConstPrimitiveVectorPtr>::registerObserver(backend);
        sensor_fusion->Subject<World>::registerObserver(ai);
        backend->Subject<SensorMsg>::registerObserver(sensor_fusion);

        // create a directory in /tmp to store replays if the parameter is set
        if (replay_logging_config->RecordReplay()->value())
        {
            fs::path replay_parent_dir(replay_logging_config->ReplayPath()->value());
            if (!fs::exists(replay_parent_dir))
            {
                fs::create_directories(replay_parent_dir);
            }

            // read the current local time and put it in a string so we can use it
            // as a directory name
            std::stringstream replay_dir_name_stream;
            std::time_t now_time =
                std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::tm now_time_tm = *std::localtime(&now_time);
            replay_dir_name_stream << std::put_time(&now_time_tm, "%Y%m%d_%H%M%S");

            fs::path replay_dir    = replay_parent_dir / replay_dir_name_stream.str();
            auto replay_logger_ptr = std::make_shared<ReplayLogger>(replay_dir);
            backend->Subject<SensorMsg>::registerObserver(replay_logger_ptr);
        }

        if (!args.headless)
        {
            visualizer = std::make_shared<ThreadedFullSystemGUI>();

            sensor_fusion->Subject<World>::registerObserver(visualizer);
            ai->Subject<AIDrawFunction>::registerObserver(visualizer);
            ai->Subject<PlayInfo>::registerObserver(visualizer);
            backend->Subject<SensorMsg>::registerObserver(visualizer);
        }

        // Wait for termination
        if (!args.headless)
        {
            // This blocks forever without using the CPU
            // Wait for the full_system to shut down before shutting
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
