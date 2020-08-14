#pragma once

#include "shared/proto/robot_status_msg.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/backend/backend.h"
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
    void onValueReceived(TbotsProto::PrimitiveSet primitives) override;
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

    // Client to listen for SSL protobufs
    SSLProtoClient ssl_proto_client;

    // ProtoMulticast** to communicate with robots
    std::unique_ptr<ThreadedProtoMulticastSender<TbotsProto::Vision>> vision_output;
    std::unique_ptr<ThreadedProtoMulticastSender<TbotsProto::PrimitiveSet>>
        primitive_output;
    std::unique_ptr<ThreadedProtoMulticastListener<TbotsProto::RobotStatus>>
        robot_msg_input;
};
