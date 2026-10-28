#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "software/networking/unix/threaded_proto_unix_listener.hpp"
#include "software/networking/unix/threaded_proto_unix_sender.hpp"
#include "software/simulation/interface/simulator_team_interface.h"

class UnixPrimitiveInterface : public SimulatorTeamInterface
{
   public:
    UnixPrimitiveInterface(std::shared_ptr<ErForceSimulator> er_force_sim,
                           std::mutex& simulator_mutex, bool is_blue_team,
                           std::string ssl_wrapper_output_path,
                           std::string robot_status_output_path,
                           std::string world_input_path,
                           std::string primitive_set_input_path)
        : SimulatorTeamInterface(std::move(er_force_sim), simulator_mutex, is_blue_team),
          world_input_path(std::move(world_input_path)),
          primitive_set_input_path(std::move(primitive_set_input_path)),
          robot_status_output(
              ThreadedProtoUnixSender<TbotsProto::RobotStatus>(robot_status_output_path)),
          ssl_wrapper_output(ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket>(
              ssl_wrapper_output_path))
    {
    }

    void startListening() override
    {
        world_input.emplace(world_input_path,
                            [this](TbotsProto::World& world) { onWorldInput(world); });
        primitive_set_input.emplace(primitive_set_input_path,
                                    [this](TbotsProto::PrimitiveSet& primitives)
                                    { onPrimitiveSetInput(primitives); });
    }

    void onWorldInput(const TbotsProto::World& input)
    {
        std::scoped_lock lock(simulator_mutex);
        team_vision = input;
    }

    void onPrimitiveSetInput(const TbotsProto::PrimitiveSet& input)
    {
        std::scoped_lock lock(simulator_mutex);
        if (is_blue_team)
            er_force_sim->setBlueRobotPrimitiveSet(
                input, std::make_unique<TbotsProto::World>(team_vision));
        else
            er_force_sim->setYellowRobotPrimitiveSet(
                input, std::make_unique<TbotsProto::World>(team_vision));
    }

    void postSimulatorTick(
        const std::vector<SSLProto::SSL_WrapperPacket>& sslPackets) override
    {
        for (const auto& packet : sslPackets)
            ssl_wrapper_output.sendProto(packet);
        if (is_blue_team)
            for (const auto& packet : er_force_sim->getBlueRobotStatuses())
                robot_status_output.sendProto(packet);
        else
            for (const auto& packet : er_force_sim->getYellowRobotStatuses())
                robot_status_output.sendProto(packet);
    }

   private:
    std::string world_input_path;
    std::string primitive_set_input_path;

    std::optional<ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>>
        primitive_set_input;
    std::optional<ThreadedProtoUnixListener<TbotsProto::World>> world_input;
    ThreadedProtoUnixSender<TbotsProto::RobotStatus> robot_status_output;
    ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket> ssl_wrapper_output;

    TbotsProto::World team_vision;
};
