#pragma once

#include <ros/ros.h>

#include <limits>

#include "ai/world/ball.h"
#include "ai/world/team.h"
#include "mrf/dongle.h"
#include "thunderbots_msgs/RobotStatus.h"

class MRFBackend
{
   public:
    /**
     * Creates a new MRFBackend.
     * Automatically connects to the dongle upon initialization.
     * 
     * @param node_handle the ROS NodeHandle of the radio_communication node
     */
    explicit MRFBackend(ros::NodeHandle& node_handle);

    ~MRFBackend();

    /**
     * IMPORTANT: Must be called in the main loop
     * Allows libusb events on the dongle to complete.
     */
    void update_dongle_events();

    /**
     * Sends the given primitives to the backend to control the robots
     *
     * @param primitives the list of primitives to send
     */
    void sendPrimitives(const std::vector<std::unique_ptr<Primitive>>& primitives);

    /**
     * Updates the detected robots from vision.
     *
     * @param robots a vector of tuples of {robot id, robot location, robot orientation}
     */
    void update_robots(std::vector<std::tuple<uint8_t, Point, Angle>> robots);

    /**
     * Updates the backend with the latest ball.
     *
     * @param b new ball
     */
    void update_ball(Ball b);

    /**
     * Sends a camera packet with the detected robots and ball.
     */
    void send_vision_packet();

    /**
     * Parses a robot status message from the dongle, and publishes it to the ROS topic.
     * 
     * @param index robot number
     * @param data the message data
     * @param len the length of the message
     * @param lqi link quality
     * @param rssi received signal strength indicator
     */
    thunderbots_msgs::RobotStatus handle_message(int index, const void* data,
                                                 std::size_t len, uint8_t lqi,
                                                 uint8_t rssi);

   private:
    MRFDongle dongle;

    ros::Publisher robot_status_publisher;
    std::vector<std::tuple<uint8_t, Point, Angle>> robots;
    Ball ball;
};
