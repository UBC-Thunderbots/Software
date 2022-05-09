#include <boost/program_options.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <numeric>

#include "proto/logging/proto_logger.h"
#include "proto/message_translation/ssl_wrapper.h"
#include "proto/play_info_msg.pb.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "shared/parameter/dynamic_parameters.pb.h"
#include "software/ai/threaded_ai.h"
#include "software/backend/backend.h"
#include "software/backend/unix_simulator_backend.h"
#include "software/constants.h"
#include "software/estop/arduino_util.h"
#include "software/gui/full_system/threaded_full_system_gui.h"
#include "software/logger/logger.h"
#include "software/multithreading/observer_subject_adapter.hpp"
#include "software/networking/threaded_proto_unix_listener.hpp"
#include "software/sensor_fusion/threaded_sensor_fusion.h"
#include "software/util/generic_factory/generic_factory.h"

int main(int argc, char** argv)
{
    // Setup dynamic parameters
    auto mutable_thunderbots_config = std::make_shared<ThunderbotsConfig>();
    auto thunderbots_config =
        std::const_pointer_cast<const ThunderbotsConfig>(mutable_thunderbots_config);
    bool help_requested =
        mutable_thunderbots_config->getMutableFullSystemMainCommandLineArgs()
            ->loadFromCommandLineArguments(argc, argv);

    std::string runtime_path =
        thunderbots_config->getFullSystemMainCommandLineArgs()->getRuntimeDir()->value();
    LoggerSingleton::initializeLogger(runtime_path);

    // TODO (#2510) remove
    bool friendly_colour_yellow =
        mutable_thunderbots_config->getMutableSensorFusionConfig()
            ->getMutableFriendlyColorYellow()
            ->setValue(thunderbots_config->getFullSystemMainCommandLineArgs()
                           ->getFriendlyColourYellow()
                           ->value());

    if (!help_requested)
    {
        auto backend = std::make_shared<UnixSimulatorBackend>(
            thunderbots_config->getBackendConfig());
        auto sensor_fusion = std::make_shared<ThreadedSensorFusion>(
            thunderbots_config->getSensorFusionConfig());
        auto ai = std::make_shared<ThreadedAI>(thunderbots_config->getAiConfig());

        // Overrides
        auto tactic_override_listener =
            ThreadedProtoUnixListener<TbotsProto::AssignedTacticPlayControlParams>(
                runtime_path + TACTIC_OVERRIDE_PATH,
                [&ai](TbotsProto::AssignedTacticPlayControlParams input) {
                    ai->overrideTactics(input);
                });

        auto play_override_listener = ThreadedProtoUnixListener<TbotsProto::Play>(
            runtime_path + PLAY_OVERRIDE_PATH,
            [&ai](TbotsProto::Play input_play) { ai->overridePlay(input_play); });

        // Connect observers
        ai->Subject<TbotsProto::PrimitiveSet>::registerObserver(backend);
        sensor_fusion->Subject<World>::registerObserver(ai);
        sensor_fusion->Subject<World>::registerObserver(backend);
        backend->Subject<SensorProto>::registerObserver(sensor_fusion);

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }

    return 0;
}
