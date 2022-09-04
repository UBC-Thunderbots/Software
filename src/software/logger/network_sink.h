#pragma once
#include <fstream>
#include <g3log/logmessage.hpp>
#include <iostream>

#include "proto/robot_log_msg.pb.h"
#include "google/protobuf/any.pb.h"
#include "software/networking/threaded_proto_udp_sender.hpp"
#include "software/logger/custom_logging_levels.h"
#include "software/logger/constants.h"
#include "proto/serialized_proto.pb.h"
#include "proto/visualization.pb.h"
#include "base64.h"

/**
 * This class acts as a custom sink for g3log. In particular, it allows us to log to
 * multicast channels from robots
 */
class NetworkSink
{
   public:
    /**
     * Creates a NetworkSink that sends udp packets to the channel provided
     *
     * @param channel The channel to join, index of NETWORK_LOGGING_MULTICAST_CHANNELS in
     * software/constants.h
     * @param interface The interface to join the multicast group on (lo, eth0, enp3s0f1,
     * etc.)
     * @param robot_id id of the robot sending the logs
     */
    NetworkSink(unsigned int channel, const std::string& interface, int robot_id);
    /**
     * This function is called on every call to LOG(). It sends a RobotLog proto on the
     * network
     *
     * @param log_entry the message received on a LOG() call
     */
    void sendToNetwork(g3::LogMessageMover log_entry);

   private:
    std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::RobotLog>> log_output;
    std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::HRVOVisualization>> serialized_proto_log_output;
    int robot_id;
};
