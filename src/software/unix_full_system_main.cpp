#include <boost/program_options.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <numeric>

#include "proto/message_translation/ssl_wrapper.h"
#include "proto/parameters.pb.h"
#include "proto/play_info_msg.pb.h"
#include "software/ai/threaded_ai.h"
#include "software/backend/backend.h"
#include "software/backend/unix_simulator_backend.h"
#include "software/constants.h"
#include "software/estop/arduino_util.h"
#include "software/logger/logger.h"
#include "software/multithreading/observer_subject_adapter.hpp"
#include "software/networking/udp/threaded_proto_udp_listener.hpp"
#include "software/networking/unix/threaded_proto_unix_listener.hpp"
#include "software/sensor_fusion/threaded_sensor_fusion.h"
#include "software/util/generic_factory/generic_factory.h"

int main(int argc, char** argv)
{
    // Setup dynamic parameters
    struct CommandLineArgs
    {
        bool help                   = false;
        std::string runtime_dir     = "/tmp/tbots";
        bool friendly_colour_yellow = false;
    };

    CommandLineArgs args;
    boost::program_options::options_description desc{"Options"};

    desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                       "Help screen");
    desc.add_options()("runtime_dir",
                       boost::program_options::value<std::string>(&args.runtime_dir),
                       "The directory to output logs and setup unix sockets.");
    desc.add_options()("friendly_colour_yellow",
                       boost::program_options::bool_switch(&args.friendly_colour_yellow),
                       "If false, friendly colour is blue");

    boost::program_options::variables_map vm;
    boost::program_options::store(parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (args.help)
    {
        std::cout << desc << std::endl;
    }

    if (!args.help)
    {
        LoggerSingleton::initializeLogger(args.runtime_dir);
        TbotsProto::ThunderbotsConfig tbots_proto;

        // Override friendly color
        tbots_proto.mutable_sensor_fusion_config()->set_friendly_color_yellow(
            args.friendly_colour_yellow);

        auto backend = std::make_shared<UnixSimulatorBackend>(args.runtime_dir);
        auto sensor_fusion =
            std::make_shared<ThreadedSensorFusion>(tbots_proto.sensor_fusion_config());
        auto ai = std::make_shared<ThreadedAi>(tbots_proto.ai_config());

        // Overrides
        auto tactic_override_listener =
            ThreadedProtoUnixListener<TbotsProto::AssignedTacticPlayControlParams>(
                args.runtime_dir + TACTIC_OVERRIDE_PATH,
                [&ai](TbotsProto::AssignedTacticPlayControlParams input) {
                    ai->overrideTactics(input);
                });

        auto play_override_listener = ThreadedProtoUnixListener<TbotsProto::Play>(
            args.runtime_dir + PLAY_OVERRIDE_PATH,
            [&ai](TbotsProto::Play input_play) { ai->overridePlay(input_play); });

        // Connect observers
        ai->Subject<TbotsProto::PrimitiveSet>::registerObserver(backend);
        sensor_fusion->Subject<World>::registerObserver(ai);
        sensor_fusion->Subject<World>::registerObserver(backend);
        backend->Subject<SensorProto>::registerObserver(sensor_fusion);
        backend->Subject<TbotsProto::ThunderbotsConfig>::registerObserver(ai);
        backend->Subject<TbotsProto::ThunderbotsConfig>::registerObserver(sensor_fusion);

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }

    return 0;
}
