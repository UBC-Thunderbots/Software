#pragma once

#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>

#include "util/logger/custom_g3log_sinks.h"

namespace Util
{
    namespace Logger
    {
        /**
         * Initializes a g3log logger for the calling program. This should only be called
         * once at the start of a program.
         */
        void init_logger()
        {
            std::unique_ptr<g3::LogWorker> logWorker{g3::LogWorker::createLogWorker()};
            // Add our custom Rosout sink so that all log messages are also published to
            // the /rosout topic. These messages are also sent to stdout and stderr
            logWorker->addSink(std::make_unique<Util::Logger::RosoutSink>(),
                               &Util::Logger::RosoutSink::ReceiveLogMessage);
            g3::initializeLogging(logWorker.get());
        }
    }  // namespace Logger
}  // namespace Util
