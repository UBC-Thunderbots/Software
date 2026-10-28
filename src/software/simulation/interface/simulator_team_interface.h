#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "software/simulation/er_force_simulator.h"

class SimulatorTeamInterface
{
   public:
    SimulatorTeamInterface(std::shared_ptr<ErForceSimulator> er_force_sim,
                           std::mutex& simulator_mutex, bool is_blue_team)
        : er_force_sim(std::move(er_force_sim)),
          simulator_mutex(simulator_mutex),
          is_blue_team(is_blue_team)
    {
    }
    SimulatorTeamInterface(const SimulatorTeamInterface&) = delete;
    virtual ~SimulatorTeamInterface()                     = default;

    /**
     * Start listening for inputs, forward them to the simulation.
     *
     * Assume mutex is already locked during call.
     */
    virtual void startListening() = 0;

    /**
     * Forward live information/feedback as necessary.
     *
     * Assume mutex is already locked during call.
     */
    virtual void postSimulatorTick(
        const std::vector<SSLProto::SSL_WrapperPacket>& sslPackets) = 0;

   protected:
    std::shared_ptr<ErForceSimulator> er_force_sim;
    std::mutex& simulator_mutex;
    bool is_blue_team;
};
