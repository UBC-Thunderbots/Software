#pragma once

#include "software/backend/backend.h"
#include "software/backend/radio/radio_output.h"
#include "software/backend/ssl_proto_client.h"
#include "shared/parameter_v2/cpp_dynamic_parameters.h"

class RadioBackend : public Backend
{
   public:
    RadioBackend(std::shared_ptr<const BackendConfig> config);

   private:
    static const int DEFAULT_RADIO_CONFIG = 0;

    void onValueReceived(TbotsProto::PrimitiveSet primitives) override;
    void onValueReceived(World world) override;

    /**
     * Convert robot_status to TbotsProto::RobotStatus and send as a SensorProto to
     * observers
     *
     * @param robot_status The RadioRobotStatus
     */
    void receiveRobotStatus(RadioRobotStatus robot_status);

    const std::shared_ptr<const SslCommunicationConfig> ssl_communication_config;

    // Client to listen for SSL protobufs
    SSLProtoClient ssl_proto_client;

    // The interface that lets us send primitives to the robots over radio
    RadioOutput radio_output;

    std::optional<World> most_recently_received_world;
    std::mutex most_recently_received_world_mutex;

    TbotsProto::PrimitiveSet most_recently_received_primitives;
    std::mutex most_recently_received_primitives_mutex;
};
