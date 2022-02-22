#pragma once

#include <atomic>
#include <thread>

#include "proto/tbots_software_msgs.pb.h"
#include "proto/vision.pb.h"
#include "proto/world.pb.h"
#include "shared/2021_robot_constants.h"
#include "software/networking/threaded_proto_unix_listener.hpp"
#include "software/networking/threaded_proto_unix_sender.hpp"
#include "software/simulation/er_force_simulator.h"


class StandaloneErForceSimulator
{
   public:
    StandaloneErForceSimulator();
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

    std::shared_ptr<ErForceSimulator> er_force_sim_;

    // Buffers
    TbotsProto::Vision blue_vision_;
    TbotsProto::Vision yellow_vision_;

    std::mutex simulator_mutex;
    std::thread simulation_thread;

    unsigned tick_debug_;
};
