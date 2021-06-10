#include <boost/program_options.hpp>
#include <experimental/filesystem>
#include <iostream>
#include <numeric>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play_info.h"
#include "software/ai/threaded_ai.h"
#include "software/backend/backend.h"
#include "software/constants.h"
#include "software/estop/arduino_util.h"
#include "software/gui/full_system/threaded_full_system_gui.h"
#include "software/logger/logger.h"
#include "software/multithreading/observer_subject_adapter.h"
#include "software/proto/logging/proto_logger.h"
#include "software/proto/message_translation/ssl_wrapper.h"
#include "software/sensor_fusion/threaded_sensor_fusion.h"
#include "software/util/design_patterns/generic_factory.h"

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


int main(int argc, char** argv)
{
    std::cout << BANNER << std::endl;

    // load command line arguments
    auto arduino_config = std::make_shared<ArduinoConfig>();
    auto args           = std::make_shared<FullSystemMainCommandLineArgs>(arduino_config);
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
            mutable_thunderbots_config->getMutableNetworkConfig()
                ->getMutableNetworkInterface()
                ->setValue(args->getInterface()->value());
        }

        // override arduino port or try to find programmatically
        if (!args->getArduinoConfig()->getPort()->value().empty())
        {
            mutable_thunderbots_config->getMutableArduinoConfig()
                ->getMutablePort()
                ->setValue(args->getArduinoConfig()->getPort()->value());
        }
        else
        {
            mutable_thunderbots_config->getMutableArduinoConfig()
                ->getMutablePort()
                ->setValue(ArduinoUtil::getArduinoPort().value_or(""));
        }

        if (args->getBackend()->value().empty())
        {
            LOG(FATAL) << "The option '--backend' is required but missing";
        }

        std::shared_ptr<Backend> backend =
            GenericFactory<std::string, Backend, BackendConfig>::create(
                args->getBackend()->value(), thunderbots_config->getBackendConfig());
        auto sensor_fusion = std::make_shared<ThreadedSensorFusion>(
            thunderbots_config->getSensorFusionConfig());
        auto ai = std::make_shared<ThreadedAI>(thunderbots_config->getAiConfig(),
                                               thunderbots_config->getAiControlConfig(),
                                               thunderbots_config->getPlayConfig());
        std::shared_ptr<ThreadedFullSystemGUI> visualizer;

        // Connect observers
        ai->Subject<TbotsProto::PrimitiveSet>::registerObserver(backend);
        sensor_fusion->Subject<World>::registerObserver(ai);
        sensor_fusion->Subject<World>::registerObserver(backend);
        backend->Subject<SensorProto>::registerObserver(sensor_fusion);
        sensor_fusion->Subject<World>::registerObserver(backend);
        if (!args->getHeadless()->value())
        {
            visualizer =
                std::make_shared<ThreadedFullSystemGUI>(mutable_thunderbots_config);

            sensor_fusion->Subject<World>::registerObserver(visualizer);
            ai->Subject<TbotsProto::PrimitiveSet>::registerObserver(visualizer);
            ai->Subject<AIDrawFunction>::registerObserver(visualizer);
            ai->Subject<PlayInfo>::registerObserver(visualizer);
            backend->Subject<SensorProto>::registerObserver(visualizer);
        }

        if (!args->getProtoLogOutputDir()->value().empty())
        {
            namespace fs = std::experimental::filesystem;
            // we want to log protos, make the parent directory and pass the
            // subdirectories to the ProtoLoggers for each message type
            fs::path proto_log_output_dir(args->getProtoLogOutputDir()->value());
            fs::create_directory(proto_log_output_dir);

            // log incoming SensorMsg
            auto sensor_msg_logger = std::make_shared<ProtoLogger<SensorProto>>(
                proto_log_output_dir / "Backend_SensorProto",
                ProtoLogger<SensorProto>::DEFAULT_MSGS_PER_CHUNK,
                [](const SensorProto& lhs, const SensorProto& rhs) {
                    return lhs.backend_received_time().epoch_timestamp_seconds() <
                           rhs.backend_received_time().epoch_timestamp_seconds();
                });
            // log outgoing PrimitiveSet
            auto primitive_set_logger =
                std::make_shared<ProtoLogger<TbotsProto::PrimitiveSet>>(
                    proto_log_output_dir / "AI_PrimitiveSet");
            backend->Subject<SensorProto>::registerObserver(sensor_msg_logger);
            ai->Subject<TbotsProto::PrimitiveSet>::registerObserver(primitive_set_logger);

            // log filtered world state
            bool friendly_colour_yellow = thunderbots_config->getSensorFusionConfig()
                                              ->getFriendlyColorYellow()
                                              ->value();
            auto friendly_team_colour =
                friendly_colour_yellow ? TeamColour::YELLOW : TeamColour::BLUE;

            auto world_to_ssl_wrapper_conversion_fn =
                [friendly_team_colour](const World& world) {
                    return *createSSLWrapperPacket(world, friendly_team_colour);
                };

            auto vision_logger =
                std::make_shared<ProtoLogger<SSLProto::SSL_WrapperPacket>>(
                    proto_log_output_dir / "SensorFusion_SSL_WrapperPacket");
            auto world_to_vision_adapter = std::make_shared<
                ObserverSubjectAdapter<World, SSLProto::SSL_WrapperPacket>>(
                world_to_ssl_wrapper_conversion_fn);
            sensor_fusion->registerObserver(world_to_vision_adapter);
            world_to_vision_adapter->registerObserver(vision_logger);
        }

        // Wait for termination
        if (!args->getHeadless()->value())
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
