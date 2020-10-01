#include <boost/program_options.hpp>
#include <iostream>
#include <numeric>

#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/ai/ai_wrapper.h"
#include "software/ai/hl/stp/play_info.h"
#include "software/backend/backend.h"
#include "software/backend/replay_logging/replay_logger.h"
#include "software/constants.h"
#include "software/gui/full_system/threaded_full_system_gui.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/sensor_fusion/threaded_sensor_fusion.h"
#include "software/util/design_patterns/generic_factory.h"

struct commandLineArgs
{
    bool help                          = false;
    std::string backend_name           = "";
    std::string network_interface_name = "";
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


int main(int argc, char **argv)
{
    std::cout << BANNER << std::endl;

    LoggerSingleton::initializeLogger();

    // load command line arguments
    auto args = MutableDynamicParameters->getMutableFullSystemMainCommandLineArgs();
    bool help_requested = args->loadFromCommandLineArguments(argc, argv);

    if (!help_requested)
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

        // TODO remove this when we move to the new dynamic parameter system
        // https://github.com/UBC-Thunderbots/Software/issues/1298
        if (!args->interface()->value().empty())
        {
            MutableDynamicParameters->getMutableNetworkConfig()
                ->mutableNetworkInterface()
                ->setValue(args->interface()->value());
        }

        if (args->backend()->value().empty())
        {
            LOG(FATAL) << "The option '--backend' is required but missing";
        }

        std::shared_ptr<Backend> backend =
            GenericFactory<std::string, Backend>::create(args->backend()->value());
        auto sensor_fusion = std::make_shared<ThreadedSensorFusion>(sensor_fusion_config);
        auto ai            = std::make_shared<AIWrapper>(ai_config, ai_control_config);
        std::shared_ptr<ThreadedFullSystemGUI> visualizer;

        // Connect observers
        ai->Subject<TbotsProto::PrimitiveSet>::registerObserver(backend);
        sensor_fusion->Subject<World>::registerObserver(ai);
        backend->Subject<SensorProto>::registerObserver(sensor_fusion);
        if (!args->headless()->value())
        {
            visualizer = std::make_shared<ThreadedFullSystemGUI>();

            sensor_fusion->Subject<World>::registerObserver(visualizer);
            ai->Subject<AIDrawFunction>::registerObserver(visualizer);
            ai->Subject<PlayInfo>::registerObserver(visualizer);
            backend->Subject<SensorProto>::registerObserver(visualizer);
            backend->Subject<TbotsProto::PrimitiveSet>::registerObserver(visualizer);
        }

        if (!args->replay_output_dir()->value().empty())
        {
            auto replay_logger =
                std::make_shared<ReplayLogger>(args->replay_output_dir()->value());
            backend->Subject<SensorProto>::registerObserver(replay_logger);
        }

        // Wait for termination
        if (!args->headless()->value())
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
