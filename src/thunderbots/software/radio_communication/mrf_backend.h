#pragma once

#include <limits>

#include "ai/world/ball.h"
#include "ai/world/team.h"
#include "mrf/dongle.h"

class MRFBackend
{
   public:
    /**
     * Creates a new MRFBackend.
     * Automatically connects to the dongle upon initialization.
     */
    explicit MRFBackend();

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

   private:
    MRFDongle dongle;
    std::vector<std::tuple<uint8_t, Point, Angle>> robots;
    Ball ball;
};
