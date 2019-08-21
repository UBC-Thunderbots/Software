#pragma once

#include <ros/ros.h>

#include <limits>

#include "ai/world/ball.h"
#include "ai/world/team.h"
#include "backend/robot_status.h"
#include "mrf/dongle.h"

class RadioOutput
{
   public:
    /**
     * Creates a new RadioOutput.
     * Automatically connects to the dongle upon initialization.
     *
     * @param config MRF configuration to start dongle in
     * @param received_robot_status_callback The callback function to call with new
     *                                       robot status messages
     */
    explicit RadioOutput(unsigned int config,
                         std::function<void(RobotStatus)> received_robot_status_callback);

    /**
     * Sends the given primitives to the backend to control the robots
     *
     * @param primitives the list of primitives to send
     */
    void sendPrimitives(const std::vector<std::unique_ptr<Primitive>>& primitives);

    /**
     * Sends a camera packet with the detected robots and ball.
     *
     * @param friendly_robots a vector of tuples of {robot id, robot location,
     *                        robot orientation}
     * @param ball
     */
    void sendVisionPacket(
        std::vector<std::tuple<uint8_t, Point, Angle>> friendly_robots, Ball ball);

    /**
     * Sends a camera packet with the detected robots and ball.
     *
     * @param friendly_team
     * @param ball
     */
    void sendVisionPacket(const Team& friendly_team, Ball ball);

   private:
    MRFDongle dongle;

    // The Annunciator that sends messages from the dongle to AI
    Annunciator annunciator;
};
