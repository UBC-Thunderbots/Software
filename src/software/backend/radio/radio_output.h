#pragma once

#include <limits>

#include "proto/tbots_software_msgs.pb.h"
#include "software/backend/radio/mrf/dongle.h"
#include "software/backend/radio/robot_status.h"
#include "software/world/ball.h"
#include "software/world/team.h"

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
    explicit RadioOutput(
        unsigned int config,
        std::function<void(RadioRobotStatus)> received_robot_status_callback);

    /**
     * Sends the given primitives to the backend to control the robots
     *
     * @param primitives the TbotsProto::PrimitiveSet to send
     */
    void sendPrimitives(const TbotsProto::PrimitiveSet& primitives);

    /**
     * Sends a camera packet with the detected robots and ball.
     *
     * @param friendly_robots a vector of tuples of {robot id, robot location,
     *                        robot orientation}
     * @param ball
     */
    void sendVisionPacket(std::vector<std::tuple<uint8_t, Point, Angle>> friendly_robots,
                          Ball ball);

    /**
     * Sends a camera packet with the detected robots and ball.
     *
     * @param friendly_team
     * @param ball
     */
    void sendVisionPacket(const Team& friendly_team, Ball ball);

   private:
    // The Annunciator that sends messages from the dongle to AI
    Annunciator annunciator;

    MRFDongle dongle;
};
