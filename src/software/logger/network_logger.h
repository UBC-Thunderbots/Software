#pragma once

#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>
#include <g3log/logmessage.hpp>
#include <g3log/logworker.hpp>

#include "software/logger/coloured_cout_sink.h"
#include "software/logger/network_sink.h"
#include "software/world/robot_state.h"

static const std::string CSV_PATH = "/tmp";

/**
 * This class acts as a Singleton that's responsible for initializing the logger.
 */
class NetworkLoggerSingleton
{
   public:
    /**
     * Initializes a g3log logger for the calling program. This should only be
     * called once at the start of a program.
     */
    static void initializeLogger(RobotId robot_id, bool enable_log_merging);

    /**
     * Updates the underlying UDP sender associated with this network sink. Useful when a
     * new FullSystem is connected.
     */
    static void replaceUdpSender(
        std::shared_ptr<ThreadedProtoUdpSender<TbotsProto::RobotLog>> new_sender);

   private:
    NetworkLoggerSingleton(RobotId robot_id, bool enable_log_merging);

    std::unique_ptr<g3::LogWorker> logWorker;
    std::unique_ptr<g3::SinkHandle<NetworkSink>> network_sink_handle;

    static std::shared_ptr<NetworkLoggerSingleton> instance;
};
