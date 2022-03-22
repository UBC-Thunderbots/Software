#pragma once

#include "proto/defending_side_msg.pb.h"
#include "proto/robot_log_msg.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/sensor_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/backend/backend.h"
#include "software/backend/ssl_proto_client.h"
#include "software/networking/threaded_proto_unix_listener.hpp"
#include "software/networking/threaded_proto_unix_sender.hpp"

class UnixSimulatorBackend : public Backend
{
   public:
    UnixSimulatorBackend(std::shared_ptr<const BackendConfig> config);

   private:
    void onValueReceived(TbotsProto::PrimitiveSet primitives) override;
    void onValueReceived(World world) override;

    const std::shared_ptr<const SensorFusionConfig> sensor_fusion_config;

    // ThreadedProtoUnix** to communicate with Thunderscope
    // Inputs
    std::unique_ptr<ThreadedProtoUnixListener<TbotsProto::RobotStatus>>
        robot_status_input;
    std::unique_ptr<ThreadedProtoUnixListener<SSLProto::SSL_WrapperPacket>>
        ssl_wrapper_input;
    std::unique_ptr<ThreadedProtoUnixListener<SSLProto::Referee>> ssl_referee_input;
    std::unique_ptr<ThreadedProtoUnixListener<SensorProto>> sensor_proto_input;

    // Outputs
    std::unique_ptr<ThreadedProtoUnixSender<TbotsProto::World>> world_output;
    std::unique_ptr<ThreadedProtoUnixSender<TbotsProto::PrimitiveSet>> primitive_output;
    std::unique_ptr<ThreadedProtoUnixSender<DefendingSideProto>> defending_side_output;
};
