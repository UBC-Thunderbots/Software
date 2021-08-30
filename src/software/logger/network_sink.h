#pragma once
#include <fstream>
#include <g3log/logmessage.hpp>
#include <iostream>

#include "software/logger/custom_logging_levels.h"
#include "software/networking/threaded_proto_udp_sender.h"
#include "shared/proto/robot_log_msg.pb.h"


/**
 * This class acts as a custom sink for g3log. In particular, it allows us to log to multicast channels
 */
class NetworkSinc
{
   public:
    /**
     * Creates a CSVSink that logs to the directory specified
     *
     * @param log_directory the directory to save files to
     */
    NetworkSinc(int channel, const std::string& interface, int robot_id);
    /**
     * This function is called on every call to LOG(CSV, filename). It appends to the
     * specified file the message in log_entry. Note for .csv files: columns are separated
     * with "," and rows are separated with new line characters.
     *
     * @param log_entry the message received on a LOG() call
     */
    void sendToNetwork(g3::LogMessageMover log_entry);

   private:
    std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::RobotLog>> log_output;
    int robot_id_;

};
