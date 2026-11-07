#pragma once

#include <mutex>

#include "proto/parameters.pb.h"
#include "proto/replay_bookmark.pb.h"
#include "proto/robot_crash_msg.pb.h"
#include "proto/robot_log_msg.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/sensor_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/validation.pb.h"
#include "proto/world.pb.h"
#include "software/backend/backend.h"
#include "software/logger/proto_logger.h"
#include "software/networking/unix/threaded_proto_unix_listener.hpp"
#include "software/networking/unix/threaded_proto_unix_sender.hpp"

class UnixSimulatorBackend : public Backend, public Subject<TbotsProto::ThunderbotsConfig>
{
   public:
    /**
     * Constructs a new UnixSimulatorBackend
     *
     * @param runtime_dir The directory to setup the unix sockets
     */
    UnixSimulatorBackend(std::string runtime_dir,
                         const std::shared_ptr<ProtoLogger>& proto_logger);

    /**
     * Get the timestamp (in seconds) of the last World received
     */
    double getLastWorldTimeSec();

   private:
    void receiveThunderbotsConfig(TbotsProto::ThunderbotsConfig request);
    void onValueReceived(TbotsProto::PrimitiveSet primitives) override;
    void onValueReceived(World world) override;

    // ThreadedProtoUnix** to communicate with Thunderscope
    // Inputs
    std::unique_ptr<ThreadedProtoUnixListener<TbotsProto::VirtualObstacles>>
        external_obstacles_list_;
    std::unique_ptr<ThreadedProtoUnixListener<TbotsProto::RobotStatus>>
        robot_status_input;
    std::unique_ptr<ThreadedProtoUnixListener<SSLProto::SSL_WrapperPacket>>
        ssl_wrapper_input;
    std::unique_ptr<ThreadedProtoUnixListener<SSLProto::Referee>> ssl_referee_input;
    std::unique_ptr<ThreadedProtoUnixListener<SensorProto>> sensor_proto_input;
    std::unique_ptr<ThreadedProtoUnixListener<TbotsProto::ThunderbotsConfig>>
        dynamic_parameter_update_request_listener;
    std::unique_ptr<ThreadedProtoUnixListener<TbotsProto::ValidationProtoSet>>
        validation_proto_set_listener;
    std::unique_ptr<ThreadedProtoUnixListener<TbotsProto::RobotLog>> robot_log_listener;
    std::unique_ptr<ThreadedProtoUnixListener<TbotsProto::RobotCrash>>
        robot_crash_listener;
    std::unique_ptr<ThreadedProtoUnixListener<TbotsProto::ReplayBookmark>>
        replay_bookmark_listener;

    // Outputs
    std::unique_ptr<ThreadedProtoUnixSender<TbotsProto::World>> world_output;
    std::unique_ptr<ThreadedProtoUnixSender<TbotsProto::PrimitiveSet>> primitive_output;
    std::unique_ptr<ThreadedProtoUnixSender<TbotsProto::ThunderbotsConfig>>
        dynamic_parameter_update_respone_sender;

    std::shared_ptr<ProtoLogger> proto_logger;

    // World protobuf sequence number counter
    uint64_t sequence_number = 0;

    // The timestamp of the last world received
    std::atomic<double> last_world_time_sec = 0;
};
