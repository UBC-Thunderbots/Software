#pragma once

#include <boost/asio.hpp>
#include <string>

#include "ai/world/team.h"
#include "backend_output/backend.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "proto/grSim_Packet.pb.h"


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

    void sendPrimitives(
        const std::vector<std::unique_ptr<Primitive>>& primitives, Team& team) override;

    /**
     * Creates a grSim Packet protobuf message given velocity information for a robot.
     * Velocities are in the Robot's local coordinate system. This function is left public
     * so that it's easily testable
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
    grSim_Packet createGrSimPacket(unsigned int robot_id, TeamColour team_colour,
                                   Vector velocity,
                                   AngularVelocity angular_velocity) const;
    
    // TODO: Implement grSim bang bang controller
    // https://github.com/UBC-Thunderbots/Software/issues/16

   private:
    /**
     * Sends a grSim packet to grSim via UDP
     *
     * @param packet the grSim packet to send
     */
    void sendGrSimPacket(const grSim_Packet& packet);


    // Variables for networking
    std::string network_address;
    unsigned short port;
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket;
    boost::asio::ip::udp::endpoint remote_endpoint;
};
