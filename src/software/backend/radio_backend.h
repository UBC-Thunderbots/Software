#pragma once

#include "software/backend/backend.h"
#include "software/backend/radio/radio_output.h"
#include "software/backend/ssl_proto_client.h"
#include "software/parameter/dynamic_parameters.h"

class RadioBackend : public Backend
{
   public:
    static const std::string name;

    RadioBackend(std::shared_ptr<const SSLCommunicationConfig> ssl_communication_config =
                     DynamicParameters->getNetworkConfig()->getSSLCommunicationConfig());

   private:
    static const int DEFAULT_RADIO_CONFIG = 0;

    void onValueReceived(PrimitiveSetMsg primitives) override;
    void onValueReceived(World world) override;

    /**
     * Convert robot_status to RobotStatusMsg and send as a SensorMsg to observers
     *
     * @param robot_status The RobotStatus
     */
    void receiveRobotStatus(RobotStatus robot_status);

    const std::shared_ptr<const SSLCommunicationConfig> ssl_communication_config;

    // Client to listen for SSL protobufs
    SSLProtoClient ssl_proto_client;

    // The interface that lets us send primitives to the robots over radio
    RadioOutput radio_output;

    std::optional<World> most_recently_received_world;
    std::mutex most_recently_received_world_mutex;

    PrimitiveSetMsg most_recently_received_primitives;
    std::mutex most_recently_received_primitives_mutex;
};
