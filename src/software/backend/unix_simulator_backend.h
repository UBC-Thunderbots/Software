#pragma once

#include "proto/defending_side_msg.pb.h"
#include "proto/parameters.pb.h"
#include "proto/robot_log_msg.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/sensor_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/backend/backend.h"
#include "software/networking/threaded_proto_unix_listener.hpp"
#include "software/networking/threaded_proto_unix_sender.hpp"

class UnixSimulatorBackend : public Backend, public Subject<TbotsProto::ThunderbotsConfig>
{
   public:
    /**
     * Constructs a new UnixSimulatorBackend
     *
     * @param runtime_dir The directory to setup the unix sockets
     */
    UnixSimulatorBackend(std::string runtime_dir);

   private:
    void receiveThunderbotsConfig(TbotsProto::ThunderbotsConfig request);
    void onValueReceived(TbotsProto::PrimitiveSet primitives) override;
    void onValueReceived(World world) override;

    // ThreadedProtoUnix** to communicate with Thunderscope
    // Inputs
    std::unique_ptr<ThreadedProtoUnixListener<TbotsProto::RobotStatus>>
        robot_status_input;
    std::unique_ptr<ThreadedProtoUnixListener<SSLProto::SSL_WrapperPacket>>
        ssl_wrapper_input;
    std::unique_ptr<ThreadedProtoUnixListener<SSLProto::Referee>> ssl_referee_input;
    std::unique_ptr<ThreadedProtoUnixListener<SensorProto>> sensor_proto_input;
    std::unique_ptr<ThreadedProtoUnixListener<TbotsProto::ThunderbotsConfig>>
        dynamic_parameter_update_request_listener;

    // Outputs
    std::unique_ptr<ThreadedProtoUnixSender<TbotsProto::World>> world_output;
    std::unique_ptr<ThreadedProtoUnixSender<TbotsProto::PrimitiveSet>> primitive_output;
    std::unique_ptr<ThreadedProtoUnixSender<DefendingSideProto>> defending_side_output;
    std::unique_ptr<ThreadedProtoUnixSender<TbotsProto::ThunderbotsConfig>>
        dynamic_parameter_update_respone_sender;
};
