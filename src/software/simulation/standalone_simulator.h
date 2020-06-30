#pragma once

#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/networking/threaded_proto_multicast_listener.h"
#include "software/networking/threaded_proto_multicast_sender.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/simulation/threaded_simulator.h"

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
     * @param msg The primitives to set on the respective team
     */
    void setYellowRobotPrimitives(PrimitiveSetMsg msg);
    void setBlueRobotPrimitives(PrimitiveSetMsg msg);

    /**
     * A helper function that sets up all networking functionality with
     * the networking information in the StandlaoneSimulatorConfig
     */
    void initNetworking();

    /**
     * Decodes a PrimitiveMsg into its primitive index and parameters
     *
     * @param msg The message to decode
     *
     * @return The primitive index and parameters for the given PrimitiveMsg
     */
    static std::pair<unsigned int, primitive_params_t> decodePrimitiveMsg(
        const PrimitiveMsg& msg);

    std::shared_ptr<const StandaloneSimulatorConfig> standalone_simulator_config;
    std::unique_ptr<ThreadedProtoMulticastSender<SSL_WrapperPacket>>
        wrapper_packet_sender;
    std::unique_ptr<ThreadedProtoMulticastListener<PrimitiveSetMsg>>
        yellow_team_primitive_listener;
    std::unique_ptr<ThreadedProtoMulticastListener<PrimitiveSetMsg>>
        blue_team_primitive_listener;
    ThreadedSimulator simulator;
};