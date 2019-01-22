#pragma once

#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>

#include "util/logger/custom_g3log_sinks.h"

namespace Util
{
    namespace Logger
    {
        /**
         * This class acts as a Singleton that's responsible for initializing the logger.
         * We use a singleton rather than a generic function in this namespace because
         * the logWorker object must stay in scope, otherwise the logger is automatically
         * destroyed. So if an "init" function is used, the logWorker goes out of scope as
         * soon as the function returns, which destroys the logger right after creating it
         *
         * The Singleton class allows us to keep the logWorker in scope for the duration
         * of the program while still providing a single function to initialize the logger
         */
        class LoggerSingleton
        {
           public:
            /**
             * Initializes a g3log logger for the calling program. This should only be
             * called once at the start of a program.
             */
            static void initializeLogger(ros::NodeHandle &node_handle)
            {
                static std::shared_ptr<LoggerSingleton> s(
                    new LoggerSingleton(node_handle));
            }


           private:
            LoggerSingleton(ros::NodeHandle &node_handle)
            {
                // Set the logger level to DEBUG so that all log messages show up
                // regardless of severity. The default is INFO, which does not show DEBUG
                // messages See http://wiki.ros.org/rosconsole for information on the
                // logger levels
                if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                                   ros::console::levels::Debug))
                {
                    ros::console::notifyLoggerLevelsChanged();
                }

                logWorker = g3::LogWorker::createLogWorker();
                // Add our custom sink to the logWorker. This custom sink logs messages
                // to both stdout and stderr, and the /rosout topic. A sink MUST be
                // provided here or the logger won't do anything.
                logWorker->addSink(
                    std::make_unique<Util::Logger::RosoutSink>(node_handle),
                    &Util::Logger::RosoutSink::ReceiveLogMessage);
                g3::initializeLogging(logWorker.get());
            }

            std::unique_ptr<g3::LogWorker> logWorker;
        };
    }  // namespace Logger
}  // namespace Util
