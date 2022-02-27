#pragma once

#include <atomic>

#include "proto/tbots_software_msgs.pb.h"
#include "proto/vision.pb.h"
#include "proto/world.pb.h"
#include "software/networking/threaded_proto_unix_listener.hpp"
#include "software/networking/threaded_proto_unix_sender.hpp"
#include "software/simulation/er_force_simulator.h"

const std::string WORLD_STATE_PATH         = "/world_state";
const std::string SSL_WRAPPER_PACKET_PATH  = "/ssl_wrapper_packet";
const std::string BLUE_ROBOT_STATUS_PATH   = "/blue_robot_status";
const std::string YELLOW_ROBOT_STATUS_PATH = "/yellow_robot_status";
const std::string SIMULATION_TICK_PATH     = "/simulation_tick";
const std::string YELLOW_VISION_PATH       = "/yellow_vision";
const std::string BLUE_VISION_PATH         = "/blue_vision";
const std::string BLUE_PRIMITIVE_SET       = "/blue_primitive_set";
const std::string YELLOW_PRIMITIVE_SET     = "/yellow_primitive_set";
class StandaloneErForceSimulator
{
   public:
    /**
     * Creates a ER force simulator and sets up the appropriate
     * communication channels (unix senders/listeners). All inputs (left) and
     * outputs (right) shown below are over unix sockets.
     *
     *
     *                        ┌────────────────────────────┐
     *   SimulatorTick        │                            │
     *   ─────────────────────►                            │
     *                        │     ER Force Simulator     │
     *                        │            Main            │
     *   WorldState           │                            │
     *   ─────────────────────►                            │ SSL_WrapperPacket
     *                        │                            ├───────────────────►
     *   Blue Primitive Set   │                            │
     *   ─────────────────────►  ┌──────────────────────┐  │ Blue Robot Status
     *   Yellow Primitive Set │  │                      │  ├───────────────────►
     *                        │  │                      │  │ Yellow Robot Status
     *                        │  │  ER Force Simulator  │  │
     *   Blue Vision          │  │                      │  │
     *   ─────────────────────►  │                      │  │
     *   Yellow Vision        │  └──────────────────────┘  │
     *                        └────────────────────────────┘
     *
     * @param base_unix_path The base path of the unix sockets
     */
    StandaloneErForceSimulator(std::string base_unix_path);
    virtual ~StandaloneErForceSimulator();

   private:
    // Listeners
    std::shared_ptr<ThreadedProtoUnixListener<TbotsProto::SimulatorTick>>
        simulation_tick_input_;
    std::shared_ptr<ThreadedProtoUnixListener<TbotsProto::WorldState>> world_state_input_;
    std::shared_ptr<ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>>
        blue_primitive_set_input_;
    std::shared_ptr<ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>>
        yellow_primitive_set_input_;
    std::shared_ptr<ThreadedProtoUnixListener<TbotsProto::Vision>> blue_vision_input_;
    std::shared_ptr<ThreadedProtoUnixListener<TbotsProto::Vision>> yellow_vision_input_;

    // Senders
    std::shared_ptr<ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket>>
        wrapper_packet_output_;
    std::shared_ptr<ThreadedProtoUnixSender<TbotsProto::RobotStatus>>
        blue_robot_status_output_;
    std::shared_ptr<ThreadedProtoUnixSender<TbotsProto::RobotStatus>>
        yellow_robot_status_output_;

    // Simulator
    std::shared_ptr<ErForceSimulator> er_force_sim_;

    // Vision Buffer
    TbotsProto::Vision blue_vision_;
    TbotsProto::Vision yellow_vision_;

    std::mutex simulator_mutex;
};
