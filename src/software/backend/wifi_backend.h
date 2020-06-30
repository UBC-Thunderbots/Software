#pragma once

#include "shared/proto/tbots_robot_msg.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/backend/backend.h"
#include "software/backend/input/network/networking/network_client.h"
#include "software/backend/ssl_proto_client.h"
#include "software/networking/threaded_proto_multicast_listener.h"
#include "software/networking/threaded_proto_multicast_sender.h"
#include "software/parameter/dynamic_parameters.h"

class WifiBackend : public Backend
{
   public:
    WifiBackend(std::shared_ptr<const NetworkConfig> network_config =
                    DynamicParameters->getNetworkConfig());

    static const std::string name;

   private:
    /**
     * This is registered as an async callback function so that it is called
     * with a new world every time one is available
     *
     * @param world The new world
     */
    void receiveWorld(World world);

    void onValueReceived(ConstPrimitiveVectorPtr primitives) override;
    void onValueReceived(World world) override;

    /**
     * Joins the specified multicast group on the vision_output, primitive_output
     * and robot_msg_input. Multicast Channel and Multicast Group are used
     * interchangeably.
     *
     * NOTE: This will terminate the existing connection on the previous channel
     * if it exists.
     *
     * @param channel The channel to join, index of MULTICAST_CHANNELS in
     * shared/constants.h
     * @param interface The interface to join the multicast group on (lo, eth0, enp3s0f1,
     * etc.)
     */
    void joinMulticastChannel(int channel, const std::string& interface);

    const std::shared_ptr<const NetworkConfig> network_config;

    // The interface with the network that lets us get new information about the world
    NetworkClient network_input;

    // Client to listen for SSL protobufs
    SSLProtoClient ssl_proto_client;

    // ProtoMulticast** to communicate with robots
    std::unique_ptr<ThreadedProtoMulticastSender<VisionMsg>> vision_output;
    std::unique_ptr<ThreadedProtoMulticastSender<PrimitiveSetMsg>> primitive_output;
    std::unique_ptr<ThreadedProtoMulticastListener<TbotsRobotMsg>> robot_msg_input;
};
