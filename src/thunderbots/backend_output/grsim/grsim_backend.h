#ifndef PROJECT_GRSIM_BACKEND_H
#define PROJECT_GRSIM_BACKEND_H
#include <boost/asio.hpp>
#include <string>
#include "backend_output/backend.h"
#include "geom/angle.h"
#include "geom/point.h"


class GrSimBackend : public Backend
{
   public:
    /**
     * Creates a new grSim backend
     *
     * @param network_address The IP address to publish grSim commands to
     * @param port The port to publish commands to
     */
    explicit GrSimBackend(std::string network_address, unsigned short port);
    ~GrSimBackend();

    /**
     * Sends the given primitives to grSim
     * @param primitives the list of primitives to send
     */
    void sendPrimitives(const std::vector<Primitive> &primitives) override;

   private:
    /**
     * Sets the velocities of a robot in grSim. The velocities are in the Robot's local
     * coordinates.
     *
     * @param robot_id The id of the robot to send the command to
     * @param team_colour_yellow Specifies if the robot to send the command to is on the
     * yellow team
     * @param velocity The velocity to set for the robot, in robot coordinates. X
     * corresponds to "forward" from the robot's perspective, and Y corresponds to the
     * left of the robot. Values are in metres per second.
     *
     *            X (forward)
     *            ^
     *            |
     *            |
     *   Y <--- Robot
     *
     * @param angular_velocity The angular velocity to set for the robot, in Radians per
     * second.
     */
    void setRobotVelocities(
        unsigned int robot_id, bool team_colour_yellow, Point velocity,
        Angle angular_velocity);


    // Variables for networking
    std::string network_address;
    unsigned short port;
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket;
    boost::asio::ip::udp::endpoint remote_endpoint;
};

#endif  // PROJECT_GRSIM_BACKEND_H
