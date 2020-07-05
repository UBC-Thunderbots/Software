#pragma once

#include "software/networking/threaded_nanopb_primitive_set_multicast_listener.h"
#include "software/networking/threaded_proto_multicast_listener.h"
#include "software/networking/threaded_proto_multicast_sender.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/simulation/threaded_simulator.h"

extern "C"
{
#include "shared/proto/tbots_software_msgs.nanopb.h"
}

/**
 * This class abstracts all simulation and networking operations for
 * a StandaloneSimulator. The StandaloneSimulator can be run as a separate
 * application on a computer or network and be interacted with by up to
 * 2 instances of an AI.
 */
class StandaloneSimulator
{
   public:
    /**
     * Creates a new StandaloneSimulator, and starts the simulation.
     *
     * @param standalone_simulator_config The config for the StandaloneSimulator
     */
    explicit StandaloneSimulator(
        std::shared_ptr<StandaloneSimulatorConfig> standalone_simulator_config);
    StandaloneSimulator() = delete;

    /**
     * Registers the given callback function. This callback function will be
     * called each time the simulation updates and a new SSL_WrapperPacket
     * is generated.
     *
     * @param callback The callback function to register
     */
    void registerOnSSLWrapperPacketReadyCallback(
        const std::function<void(SSL_WrapperPacket)>& callback);

    /**
     * Adds robots to predefined locations on the field
     */
    void setupInitialSimulationState();

   private:
    /**
     * Sets the primitives being simulated by the robots on the respective team
     *
     * @param primitive_set_msg The set of primitives to run on the respective team
     */
    void setYellowRobotPrimitives(PrimitiveSetMsg primitive_set_msg);
    void setBlueRobotPrimitives(PrimitiveSetMsg primitive_set_msg);

    /**
     * A helper function that sets up all networking functionality with
     * the networking information in the StandlaoneSimulatorConfig
     */
    void initNetworking();

    std::shared_ptr<const StandaloneSimulatorConfig> standalone_simulator_config;
    std::unique_ptr<ThreadedNanoPbPrimitiveSetMulticastListener>
        yellow_team_primitive_listener, blue_team_primitive_listener;
    std::unique_ptr<ThreadedProtoMulticastSender<SSL_WrapperPacket>>
        wrapper_packet_sender;
    ThreadedSimulator simulator;
};
