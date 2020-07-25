#pragma once
#include "shared/proto/tbots_robot_msg.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/backend/backend.h"
#include "software/backend/replay_logging/replay_reader.h"
#include "software/backend/ssl_proto_client.h"
#include "software/networking/threaded_proto_multicast_listener.h"
#include "software/networking/threaded_proto_multicast_sender.h"
#include "software/parameter/dynamic_parameters.h"

class ReplayBackend : public Backend
{
   public:
    explicit ReplayBackend(
        std::shared_ptr<const ReplayBackendConfig> replay_backend_config =
            DynamicParameters->getReplayBackendConfig());

    static const std::string name;

   private:
    void onValueReceived(ConstPrimitiveVectorPtr primitives) override;
    void onValueReceived(World world) override;
    void continuouslyPullFromReplayFiles();

    ReplayReader replay_reader;
    std::thread pull_from_replay_thread;
    std::optional<std::chrono::duration<double>> last_msg_received_time;
    std::optional<std::chrono::time_point<std::chrono::steady_clock>>
        last_msg_replayed_time;
};
