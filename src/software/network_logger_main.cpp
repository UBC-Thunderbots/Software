#include <boost/program_options.hpp>
#include <iostream>
#include <numeric>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "shared/proto/robot_log_msg.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/constants.h"
#include "software/networking/threaded_proto_udp_listener.h"

/*
 * This standalone program listens for RobotLog protos on the specified ip addres2079
 * and prints them
 */

void robotLogReceiver(TbotsProto::RobotLog log)
{
    LEVELS level(INFO);

    if (TbotsProto::LogLevel_Name(log.log_level()) == "DEBUG")
    {
        level = DEBUG;
    }
    else if (TbotsProto::LogLevel_Name(log.log_level()) == "WARNING" ||
             TbotsProto::LogLevel_Name(log.log_level()) == "FATAL")
    {
        //log FATAL as WARNING to prevent program from exiting
        level = WARNING;
    }

    LOG(level) << "[ROBOT " << log.robot_id() << " " << LogLevel_Name(log.log_level())
               << "]"
               << "[" << log.file_name() << ":" << log.line_number()
               << "]: " << log.log_msg() << std::endl;
}

int main(int argc, char **argv)
{
    // load command line arguments
    auto args           = std::make_shared<NetworkLoggerMainCommandLineArgs>();
    bool help_requested = args->loadFromCommandLineArguments(argc, argv);

    auto logWorker               = g3::LogWorker::createLogWorker();
    auto colour_cout_sink_handle = logWorker->addSink(
        std::make_unique<ColouredCoutSink>(false), &ColouredCoutSink::displayColouredLog);
    g3::initializeLogging(logWorker.get());

    if (!help_requested)
    {
        int channel           = args->getChannel()->value();
        std::string interface = args->getInterface()->value();


        auto log_input = std::make_unique<ThreadedProtoUdpListener<TbotsProto::RobotLog>>(
            std::string(NETWORK_LOGGING_MULTICAST_CHANNELS[channel]) + "%" + interface,
            NETWORK_LOGS_PORT, std::function(robotLogReceiver), true);


        LOG(INFO) << "Network logger listenting on channel "
                  << NETWORK_LOGGING_MULTICAST_CHANNELS[channel] << " and interface "
                  << interface << std::endl;

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }

    return 0;
}
