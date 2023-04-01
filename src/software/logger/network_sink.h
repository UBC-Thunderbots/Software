#pragma once
#include <fstream>
#include <g3log/logmessage.hpp>
#include <iostream>

#include "proto/robot_log_msg.pb.h"
#include "software/logger/log_merger.h"
#include "software/networking/threaded_proto_udp_sender.hpp"


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
     * network and merges repeated messages.
     *
     * @param log_entry the message received on a LOG() call
     */
    void sendToNetwork(g3::LogMessageMover log_entry);

    /**
     * Send a single log to the network, without merging.
     *
     * @param log the LogMessage to send
     */
    void sendOneToNetwork(g3::LogMessage log);

   private:
    std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::RobotLog>> log_output;
    int robot_id;
    LogMerger log_merger;
};
