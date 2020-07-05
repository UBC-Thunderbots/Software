#pragma once

#include <boost/asio.hpp>
#include <string>

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/primitive/primitive.h"
#include "software/proto/grSim_Packet.pb.h"
#include "software/world/ball.h"
#include "software/world/team.h"


class GrSimOutput
{
   public:
    /**
     * Creates a new grSim backend
     *
     * @param network_address The IP address to publish grSim commands to
     * @param port The port to publish commands to
     */
    explicit GrSimOutput(std::string network_address, unsigned short port,
                         std::shared_ptr<const SensorFusionConfig> config);

    ~GrSimOutput();

    /**
     * Sends the given primitives to be simulated in grSim
     *
     * @param primitives the list of primitives to send
     * @param friendly_team A Team object containing the latest data for the friendly team
     * @param ball The ball
     */
    void sendPrimitives(const std::vector<std::unique_ptr<Primitive>>& primitives,
                        const Team& friendly_team, const Ball& ball);

    /**
     * Creates a grSim Packet protobuf message given velocity information for a robot.
     * Velocities are in the Robot's local coordinate system. This function is left public
     * so it is easy to test.
     *
     * @param robot_id The id of the robot to send the command to
     * @param is_yellow Specifies if the robot to send the command to is on the
     * yellow team
     * @param robot_velocity The velocity to set for the robot, in robot coordinates. X
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
     * second. A positive value results in counter-clockwise rotation, and a negative
     * values result in clockwise rotation
     * @param kick_speed_meters_per_second How hard to kick/chip the ball, in
     * meters/second. If chip is false, the ball is kicked along the ground. If chip is
     * true, the ball is chipped in the air
     * @param chip Whether or not to chip the ball. If chip is true, the ball is chipped
     * in the air, otherwise it is kicked along the ground
     * @param dribbler_on Whether or not to turn on the dribbler. If dribbler_on is true,
     * the dribbler will be turned on, otherwise the dribbler remains off. Dribbler speed
     * cannot be controlled in grSim
     */
    grSim_Packet createGrSimPacketWithRobotVelocity(unsigned int robot_id, bool is_yellow,
                                                    Vector robot_velocity,
                                                    AngularVelocity angular_velocity,
                                                    double kick_speed_meters_per_second,
                                                    bool chip, bool dribbler_on) const;

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
    std::shared_ptr<const SensorFusionConfig> config;
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket;
    boost::asio::ip::udp::endpoint remote_endpoint;
};
