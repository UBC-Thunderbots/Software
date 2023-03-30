#include <boost/program_options.hpp>
#include <iostream>
#include <numeric>

#include "proto/parameters.pb.h"
#include "proto/robot_log_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/constants.h"
#include "software/networking/threaded_proto_udp_listener.hpp"

/*
 * This standalone program listens for RobotLog protos on the specified ip address
 * and logs them
 */

void logFromNetworking(TbotsProto::RobotLog log)
{
    LEVELS level(INFO);

    if (TbotsProto::LogLevel_Name(log.log_level()) == "DEBUG")
    {
        level = DEBUG;
    }
    else if (TbotsProto::LogLevel_Name(log.log_level()) == "WARNING" ||
             TbotsProto::LogLevel_Name(log.log_level()) == "FATAL")
    {
        // log FATAL as WARNING to prevent program from exiting
        // note that the level of the RobotLog will itself be printed
        level = WARNING;
    }

    LOG(level) << "[ROBOT " << log.robot_id() << " " << LogLevel_Name(log.log_level())
               << "]"
               << "[" << log.file_name() << ":" << log.line_number()
               << "] " << log.created_timestamp().epoch_timestamp_seconds() << ": " << log.log_msg();
}

int main(int argc, char **argv)
{
    // TODO: Update to allow filtering of robots, and changing multicast channel
    // load command line arguments
    auto args           = std::make_shared<NetworkLogListenerMainCommandLineArgs>();
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
            std::string(ROBOT_MULTICAST_CHANNELS[channel]) + "%" + interface,
            ROBOT_LOGS_PORT, std::function(logFromNetworking), true);


        LOG(INFO) << "Network logger listening on channel "
                  << ROBOT_MULTICAST_CHANNELS[channel] << " and interface "
                  << interface << std::endl;

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }

    return 0;
}
