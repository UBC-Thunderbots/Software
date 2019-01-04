#pragma once

#include <string>
#include <optional>
#include <ros/ros.h>
#include <chrono>
#include <thread>

namespace RosTest {
    /**
     * This is a utility function primarily designed for use in RosTests. Given a
     * NodeHandle and topic name, this function blocks until a message is received on the
     * topic, in which case the message pointer is returned, or the timeout expires.
     * If the timeout expires, a runtime error is thrown.
     *
     * @tparam T A pointer to the message class of the topic.
     * Eg std_msgs::String::ConstPtr
     * @param node_handle A reference to the NodeHandle object that will be subscribing to
     * the topic
     * @param topic_name A string containing the name of the topic to subscribe to
     * @param timeout The number of seconds the function will wait for a message to be
     * received on the topic before an exception is thrown
     * @throw std::runtime_error If no messages are received on the topic before the
     * timout expires
     * @return A pointer to the message class of the topic
     */
    template <typename T>
    T waitForMessageOnTopic(ros::NodeHandle &node_handle, std::string topic_name, std::chrono::seconds timeout=std::chrono::seconds(10))
    {
        std::optional<T> msg_ptr;

        // Do NOT use the 'auto' keyword here. The type is uses fails compilation.
        boost::function<void (T)> callback = [&] (T msg) {
            msg_ptr = msg;
        };

        // We need to store the subscriber in a variable to it doesn't immediately go out of scope and get deconstructed
        auto sub = node_handle.subscribe<T>(topic_name, 1, callback);

        std::chrono::time_point start = std::chrono::steady_clock::now();
        while(!msg_ptr && (std::chrono::steady_clock::now() - start) < timeout)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            ros::spinOnce();
        }

        if(msg_ptr) {
            return msg_ptr.value();
        }

        throw std::runtime_error(std::string("Error: Timed out while waiting for message on topic ") + topic_name);
    }
}
