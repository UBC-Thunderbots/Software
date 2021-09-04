#pragma once

#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>
#include <g3log/logmessage.hpp>
#include <g3log/logworker.hpp>

#include "software/logger/network_sink.h"

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
    static void initializeLogger(int channel, const std::string& interface, int robot_id)
    {
        static std::shared_ptr<NetworkLoggerSingleton> s(new NetworkLoggerSingleton(channel, interface, robot_id));
    }


private:
    NetworkLoggerSingleton(int channel, const std::string& interface, int robot_id)
    {
        logWorker = g3::LogWorker::createLogWorker();

        auto filtered_log_rotate_sink_handle = logWorker->addSink(
                std::make_unique<struct NetworkSinc>(
                        channel, interface, robot_id),
                &NetworkSinc::sendToNetwork);

        g3::initializeLogging(logWorker.get());
    }

    std::unique_ptr<g3::LogWorker> logWorker;
};
