#pragma once
#include "shared/proto/robot_status_msg.pb.h"
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
    explicit ReplayBackend(const std::string& replay_input_dir =
                               DynamicParameters->getFullSystemMainCommandLineArgs()
                                   ->replay_input_dir()
                                   ->value());

   private:
    void onValueReceived(TbotsProto::PrimitiveSet primitives) override;
    void onValueReceived(World world) override;
    void continuouslyPullFromReplayFiles();

    ReplayReader replay_reader;
    // a thread that continuously pulls from replay data files and emits them to the
    // observers of this class
    std::thread pull_from_replay_thread;
    std::optional<std::chrono::duration<double>> last_msg_received_time;
    std::optional<std::chrono::time_point<std::chrono::steady_clock>>
        last_msg_replayed_time;
};
