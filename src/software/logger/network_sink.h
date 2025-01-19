#pragma once
#include <fstream>
#include <g3log/logmessage.hpp>
#include <iostream>

#include "proto/robot_log_msg.pb.h"
#include "proto/visualization.pb.h"
#include "software/logger/log_merger.h"
#include "software/networking/udp/threaded_proto_udp_sender.hpp"


/**
 * This class acts as a custom sink for g3log. In particular, it allows us to log to
 * multicast channels from robots
 */
class NetworkSink
{
   public:
    /**
     * Creates a NetworkSink that sends UDP packets.
     *
     * @param robot_id id of the robot sending the logs
     * @param enable_log_merging Whether to merge repeated log message or not
     */
    NetworkSink(int robot_id, bool enable_log_merging);

    /**
     * Replaces the underlying UDP sender with a new one. Intended to be used when a new
     * Full-System node is connected.
     *
     * @param new_sender the new UDP sender to use
     */
    void replaceUdpSender(
        std::shared_ptr<ThreadedProtoUdpSender<TbotsProto::RobotLog>> new_sender);

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
    void sendOneLogToNetwork(const g3::LogMessage& log);

   private:
    int robot_id;
    LogMerger log_merger;

    std::mutex robot_log_sender_mutex;
    std::optional<std::shared_ptr<ThreadedProtoUdpSender<TbotsProto::RobotLog>>>
        robot_log_sender;
};
