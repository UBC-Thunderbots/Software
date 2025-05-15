#include <Tracy.hpp>
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
#include "software/logger/proto_logger.h"
#include "software/multithreading/observer_subject_adapter.hpp"
#include "software/networking/udp/threaded_proto_udp_listener.hpp"
#include "software/networking/unix/threaded_proto_unix_listener.hpp"
#include "software/sensor_fusion/threaded_sensor_fusion.h"
#include "software/util/generic_factory/generic_factory.h"

// ProtoLogger has to be defined as a global variable so that it can be accessed by the
// cleanup callback.
std::shared_ptr<ProtoLogger> proto_logger;

/**
 * Signal handler which attempts to cleanly shutdown the program.
 *
 * @note This function is not guaranteed to clean up all resources since it calls
 * non signal-safe functions (e.g. std::shared_ptr, std::cout, exit, etc.). The primary
 * purpose of this function is to *try* to flush and stop the ProtoLogger.
 *
 * @param signal_num The signal number that was caught
 */
void cleanup(int signal_num)
{
    if (proto_logger)
    {
        proto_logger->flushAndStopLogging();
    }

    // Program has cleaned up core resources, so we can safely exit
    exit(0);
}

int main(int argc, char** argv)
{
    // Setup dynamic parameters
    struct CommandLineArgs
    {
        bool help                   = false;
        std::string runtime_dir     = "/tmp/tbots";
        std::string log_name        = std::string();
        bool friendly_colour_yellow = false;
        bool ci                     = false;
    };

    CommandLineArgs args;
    boost::program_options::options_description desc{"Options"};

    desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                       "Help screen");
    desc.add_options()("runtime_dir",
                       boost::program_options::value<std::string>(&args.runtime_dir),
                       "The directory to output logs and setup unix sockets.");
    desc.add_options()("log_name",
                       boost::program_options::value<std::string>(&args.log_name),
                       "The directory name to contain proto logs.");
    desc.add_options()("friendly_colour_yellow",
                       boost::program_options::bool_switch(&args.friendly_colour_yellow),
                       "If false, friendly colour is blue");
    desc.add_options()(
        "ci", boost::program_options::bool_switch(&args.ci),
        "If true, then the World timestamp will be used to as the time provider for ProtoLogger");

    boost::program_options::variables_map vm;
    boost::program_options::store(parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (args.help)
    {
        std::cout << desc << std::endl;
    }

    if (!args.help)
    {
        if (args.friendly_colour_yellow)
        {
            TracySetProgramName("Thunderbots: Blue");
        }
        else
        {
            TracySetProgramName("Thunderbots: Yellow");
        }

        std::function<double()> time_provider;
        if (!args.ci)
        {
            // Return the current time since epoch in seconds
            time_provider = []() {
                return std::chrono::duration<double>(
                           std::chrono::system_clock::now().time_since_epoch())
                    .count();
            };
        }
        else
        {
            // Default to returning 0 and update the time_provider with the World
            // timestamp once the backend is set up
            time_provider = []() { return 0; };
        }

        if (args.log_name.empty())
        {
            proto_logger = std::make_shared<ProtoLogger>(args.runtime_dir, time_provider,
                                                     args.friendly_colour_yellow);
        }
        else
        {
            proto_logger = std::make_shared<ProtoLogger>(args.runtime_dir, time_provider,
                                                    args.friendly_colour_yellow, args.log_name);
        }

        LoggerSingleton::initializeLogger(args.runtime_dir, proto_logger);
        TbotsProto::ThunderbotsConfig tbots_proto;

        // Override friendly color
        tbots_proto.mutable_sensor_fusion_config()->set_friendly_color_yellow(
            args.friendly_colour_yellow);

        auto backend =
            std::make_shared<UnixSimulatorBackend>(args.runtime_dir, proto_logger);
        if (args.ci)
        {
            // Update the time provider for ProtoLogger
            proto_logger->updateTimeProvider(
                [&backend]() { return backend->getLastWorldTimeSec(); });
        }

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

        // Handle some of the signals that we manually send when we want to shut down full
        // system cleanly. SIGTERM is sent by Thunderscope to stop full system
        std::signal(SIGTERM, cleanup);
        // SIGINT is sent by the user when they Ctrl+C in the terminal
        std::signal(SIGINT, cleanup);

        // Handle the rest of the catch-able signals to ensure we have flushed proto
        // logger before exiting, so we can debug the cause of the error.
        std::signal(SIGSEGV, cleanup);
        std::signal(SIGABRT, cleanup);
        std::signal(SIGFPE, cleanup);
        std::signal(SIGILL, cleanup);

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }

    return 0;
}
