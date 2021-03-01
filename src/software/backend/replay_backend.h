#pragma once
#include "shared/parameter_v2/cpp_dynamic_parameters.h"
#include "shared/proto/robot_status_msg.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/backend/backend.h"
#include "software/backend/ssl_proto_client.h"
#include "software/networking/threaded_proto_multicast_listener.h"
#include "software/networking/threaded_proto_multicast_sender.h"
#include "software/proto/logging/proto_log_reader.h"

class ReplayBackend : public Backend
{
   public:
    explicit ReplayBackend(std::shared_ptr<const BackendConfig> config);

   private:
    void onValueReceived(TbotsProto::PrimitiveSet primitives) override;
    void onValueReceived(World world) override;
    void continuouslyPullFromReplayFiles();

    static constexpr std::chrono::duration<double> CHECK_LAST_PRIMITIVE_TIME_DURATION =
        std::chrono::duration<double>(0.1);
    static constexpr std::chrono::duration<double> LAST_PRIMITIVE_TO_SHUTDOWN_DURATION =
        std::chrono::duration<double>(1.0);

    ProtoLogReader replay_reader;
    // a thread that continuously pulls from replay data files and emits them to the
    // observers of this class
    std::thread pull_from_replay_thread;
    std::optional<std::chrono::duration<double>> last_msg_received_time;
    std::optional<std::chrono::time_point<std::chrono::steady_clock>>
        last_msg_replayed_time;

    std::optional<std::chrono::time_point<std::chrono::steady_clock>>
        last_primitive_received_time;
    std::mutex last_primitive_received_time_mutex;
};
