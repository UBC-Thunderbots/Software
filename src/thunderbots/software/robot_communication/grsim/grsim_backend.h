#pragma once

#include <boost/asio.hpp>
#include <string>

#include "ai/world/team.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "proto/grSim_Packet.pb.h"
#include "robot_communication/backend.h"


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
        const std::vector<std::unique_ptr<Primitive>>& primitives) override;

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
     * second. A negative value results in counter-clockwise rotation, and positive
     * values result in clockwise rotation
     * @param kick_speed_meters_per_second How hard to kick/chip the ball, in
     * meters/second. If chip is false, the ball is kicked along the ground. If chip is
     * true, the ball is chipped in the air
     * @param chip Whether or not to chip the ball. If chip is true, the ball is chipped
     * in the air, otherwise it is kicked along the ground
     * @param dribbler_on Whether or not to turn on the dribbler. If dribbler_on is true,
     * the dribbler will be turned on, otherwise the dribbler remains off
     */
    grSim_Packet createGrSimPacket(unsigned int robot_id, TeamColour team_colour,
                                   Vector velocity, AngularVelocity angular_velocity,
                                   double kick_speed_meters_per_second, bool chip,
                                   bool dribbler_on) const;

    /**
     * Helper function that updates the backend friendly team object
     * @param new_friendly_team The updated instance of the friendly team
     */
    void updateBackendTeam(const Team& new_friendly_team)
    {
        team = new_friendly_team;
    }

   private:
    // construct default team
    Team team = Team(std::chrono::milliseconds(1000));

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
