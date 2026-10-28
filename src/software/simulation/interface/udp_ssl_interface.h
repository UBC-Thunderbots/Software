#pragma once

#include <memory>
#include <mutex>
#include <optional>

#include "software/logger/logger.h"
#include "software/networking/udp/threaded_proto_udp_bidirectional.hpp"
#include "software/simulation/interface/simulator_team_interface.h"

class UdpSslInterface : public SimulatorTeamInterface
{
   public:
    UdpSslInterface(std::shared_ptr<ErForceSimulator> er_force_sim,
                    std::mutex& simulator_mutex, bool is_blue_team,
                    unsigned short robot_control_port)
        : SimulatorTeamInterface(std::move(er_force_sim), simulator_mutex, is_blue_team),
          robot_control_port(robot_control_port)
    {
    }

    void startListening() override
    {
        // mutex is locked, so this won't race
        robot_control_input.emplace(robot_control_port,
                                    [this](SSLSimulationProto::RobotControl& command,
                                           const boost::asio::ip::udp::endpoint& sender)
                                    { onRobotControl(command, sender); });
    }

    void onRobotControl(const SSLSimulationProto::RobotControl& command,
                        const boost::asio::ip::udp::endpoint& sender)
    {
        std::scoped_lock lock(simulator_mutex);
        const auto response = is_blue_team ? er_force_sim->sendBlueSSLControl(command)
                                           : er_force_sim->sendYellowSSLControl(command);
        try
        {
            robot_control_input->sendTo(response, sender);
        }
        catch (const boost::system::system_error& error)
        {
            LOG(WARNING) << "Failed to send SSL robot feedback: " << error.what();
        }
    }

    // nothing to forward, udp bots share the same common input
    void postSimulatorTick(
        const std::vector<SSLProto::SSL_WrapperPacket>& sslPackets) override
    {
    }

   private:
    unsigned short robot_control_port;
    std::optional<ThreadedProtoUdpBidirectional<SSLSimulationProto::RobotControl,
                                                SSLSimulationProto::RobotControlResponse>>
        robot_control_input;
};
