#pragma once

/**
 * This file contains custom sinks for g3log. Custom sinks allow us to control how the
 * logs are processed, such as sending them to stdout or to the /rosout topic
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <g3log/logmessage.hpp>
#include <string>

#include "util/constants.h"
#include "util/logger/custom_logging_levels.h"

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
            ros::Publisher robot_status_publisher;

            RosoutSink(ros::NodeHandle &node_handle)
            {
                // The buffer queue for this topic is larger than our other ones so that
                // we don't lose any messages if many are sent at once. The robot status
                // messages do not have the same strict time constraints as the
                // logic-based part of the system, so ensuring no messages are lost
                // is more important.
                //
                // This publisher has latching enabled primarily for the purposes of the
                // ROSTests. If it is disabled the test subscriber will miss the published
                // test messages.
                robot_status_publisher = node_handle.advertise<std_msgs::String>(
                    Util::Constants::ROBOT_STATUS_TOPIC, 100, true);
            }

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
                else if (g3log_log_level.value == ROBOT_STATUS.value)
                {
                    // Send the message to a special topic. The visualizer will listen
                    // to this topic so it can distinguish robot status messages and
                    // display them differently
                    std_msgs::String msg;
                    msg.data = log_text;
                    robot_status_publisher.publish(msg);

                    // We still log these messages to stdout and /rosout so they can be
                    // see even when the Visualizer isn't running
                    ROS_INFO("%s", log_text.c_str());
                }
                else
                {
                    throw std::invalid_argument("Unrecognized value for g3log_log_level");
                }
            }
        };
    }  // namespace Logger
}  // namespace Util
