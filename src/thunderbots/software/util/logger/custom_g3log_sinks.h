#pragma once

/**
 * This file contains custom sinks for g3log. Custom sinks allow us to control how the
 * logs are processed, such as sending them to stdout or to the /rosout topic
 */

#include <ros/ros.h>

#include <g3log/logmessage.hpp>
#include <string>

namespace Util
{
    namespace Logger
    {
        /*
         * This struct defines a custom sink for g3log that sends output to the /rosout
         * topic via the rosconsole. This also sends logs to stdout and stderr depending
         * on the log level.
         *
         * See: https://github.com/KjellKod/g3sinks/blob/master/snippets/ColorCoutSink.hpp
         * for defining custom sinks
         *
         * See: http://wiki.ros.org/roscpp/Overview/Logging
         * for the rosconsole API
         */
        struct RosoutSink
        {
            /**
             * This function is called to process a log entry every time this log sink
             * receives a new log message
             *
             * @param logEntry The log entry to process
             */
            void ReceiveLogMessage(g3::LogMessageMover logEntry)
            {
                auto g3log_log_level = logEntry.get()._level;
                std::string log_text = logEntry.get().toString();

                if (g3log_log_level.value == INFO.value)
                {
                    ROS_INFO("%s", log_text.c_str());
                }
                else if (g3log_log_level.value == DEBUG.value)
                {
                    ROS_DEBUG("%s", log_text.c_str());
                }
                else if (g3log_log_level.value == WARNING.value)
                {
                    ROS_WARN("%s", log_text.c_str());
                }
                else if (g3log_log_level.value == FATAL.value)
                {
                    ROS_ERROR("%s", log_text.c_str());
                }
                else
                {
                    throw std::invalid_argument("Unrecognized value for g3log_log_level");
                }
            }
        };
    }  // namespace Logger
}  // namespace Util
