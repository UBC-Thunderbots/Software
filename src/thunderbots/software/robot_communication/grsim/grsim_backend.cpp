#include "robot_communication/grsim/grsim_backend.h"

#include <chrono>
#include <iostream>
#include <optional>
#include <utility>

#include "ai/primitive/move_primitive.h"
#include "ai/primitive/primitive.h"
#include "ai/world/team.h"
#include "proto/grSim_Commands.pb.h"
#include "robot_communication/grsim/motion_controller.h"
#include "shared/constants.h"

using namespace boost::asio;

GrSimBackend::GrSimBackend(std::string network_address, unsigned short port)
    : network_address(network_address), port(port), socket(io_service)
{
    socket.open(ip::udp::v4());
    remote_endpoint = ip::udp::endpoint(ip::address::from_string(network_address), port);
}

GrSimBackend::~GrSimBackend()
{
    socket.close();
}

grSim_Packet GrSimBackend::createGrSimPacket(unsigned int robot_id,
                                             TeamColour team_colour, Vector velocity,
                                             AngularVelocity angular_velocity,
                                             double kick_speed_meters_per_second,
                                             bool chip, bool dribbler_on) const
{
    grSim_Packet packet;

    packet.mutable_commands()->set_isteamyellow(team_colour == YELLOW);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();

    command->set_id(robot_id);

    // We set a robot velocity, not individual wheel velocities
    command->set_wheelsspeed(false);

    command->set_veltangent(static_cast<float>(velocity.x()));
    command->set_velnormal(static_cast<float>(velocity.y()));
    command->set_velangular(static_cast<float>(angular_velocity.toRadians()));

    command->set_kickspeedx(static_cast<float>(kick_speed_meters_per_second));
    command->set_kickspeedz(
        static_cast<float>(chip ? kick_speed_meters_per_second : 0.0));
    command->set_spinner(dribbler_on);

    return packet;
}

void GrSimBackend::sendGrSimPacket(const grSim_Packet& packet)
{
    boost::system::error_code err;
    socket.send_to(
        buffer(packet.SerializeAsString(), static_cast<size_t>(packet.ByteSize())),
        remote_endpoint, 0, err);
}


void GrSimBackend::sendPrimitives(
    const std::vector<std::unique_ptr<Primitive>>& primitives)
{
    std::vector<grSim_Packet> grsim_packets;

    MotionController::Velocity robot_velocities;
    grSim_Packet grsim_packet;


    // initial timestamp for bang-bang set as current time
    static auto bangbang_timestamp = std::chrono::steady_clock::now();

    std::chrono::duration<double> delta_time;

    for (auto& prim : primitives)
    {
        MovePrimitive movePrim = dynamic_cast<MovePrimitive&>(*prim);

        // get the current time right before
        // running bang-bang to get a time-delta for acceleration
        delta_time = std::chrono::steady_clock::now() - bangbang_timestamp;

        // Motion controller determines the speeds to send to the robots based on their
        // state, maximum linear and angular accelerations, and the robots target
        // destination location/orientation and speed
        robot_velocities = MotionController::bangBangVelocityController(
            *team.getRobotById(movePrim.getRobotId()), movePrim.getDestination(),
            movePrim.getFinalSpeed(), movePrim.getFinalAngle(), delta_time.count());

        // send the velocity data via grsim_packet
        grsim_packet = createGrSimPacket(
            movePrim.getRobotId(), YELLOW, robot_velocities.linear_velocity,
            robot_velocities.angular_velocity, 0, false, false);
    }

    // timestamp of when the motion controller was last run (to be used for calculating
    // delta_time in the future)
    bangbang_timestamp = std::chrono::steady_clock::now();

    sendGrSimPacket(grsim_packet);
}
