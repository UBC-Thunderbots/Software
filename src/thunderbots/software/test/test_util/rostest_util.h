#pragma once

#include <ros/ros.h>

#include <chrono>
#include <optional>
#include <string>
#include <thread>

#include "util/timestamp.h"

namespace RosTestUtil
{
    /**
     * This is a utility function primarily designed for use in RosTests. Given a
     * NodeHandle and topic name, this function blocks until a message is received on the
     * topic, in which case the message pointer is returned, or the timeout expires.
     * If the timeout expires, a runtime error is thrown.
     *
     * Because this function blocks while waiting to receive messages, publishers must be
     * set up and test messages must be published before this function is called. In
     * situations where the publisher will only be publishing a single message, the
     * publisher will need to have latching enabled, otherwise the subscriber created in
     * this function will be too late to receive the published message.
     *
     * @tparam T A pointer to the message class of the topic.
     * Eg std_msgs::String::ConstPtr
     * @param node_handle A reference to the NodeHandle object that will be subscribing to
     * the topic
     * @param topic_name A string containing the name of the topic to subscribe to
     * @param timeout The duration the function will wait for a message to be
     * received on the topic before an exception is thrown
     * @throw std::runtime_error If no messages are received on the topic before the
     * timout expires
     * @return A pointer to the message class of the topic
     */
    template <typename T>
    T waitForMessageOnTopic(ros::NodeHandle &node_handle, std::string topic_name,
                            Duration timeout = Duration::fromSeconds(10))
    {
        std::optional<T> msg_ptr;

        // Do NOT use the 'auto' keyword here. The type it auto resolves to
        // fails compilation
        boost::function<void(T)> callback = [&](T msg) { msg_ptr = msg; };

        // We need to store the subscriber in a variable to it doesn't immediately go out
        // of scope and get deconstructed
        auto sub = node_handle.subscribe<T>(topic_name, 1, callback);

        std::chrono::time_point start = std::chrono::steady_clock::now();
        while (!msg_ptr && std::chrono::duration_cast<std::chrono::seconds>(
                               std::chrono::steady_clock::now() - start)
                                   .count() < timeout.getSeconds())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            ros::spinOnce();
        }

        if (msg_ptr)
        {
            return msg_ptr.value();
        }

        throw std::runtime_error(
            std::string("Error: Timed out while waiting for message on topic ") +
            topic_name);
    }
}  // namespace RosTestUtil
