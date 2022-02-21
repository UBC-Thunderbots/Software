#include "software/simulation/standalone_er_force_simulator.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/time/duration.h"

StandaloneErForceSimulator::StandaloneErForceSimulator()
{
    tick_debug_ = 0;

    world_state_input_.reset(new ThreadedProtoUnixListener<TbotsProto::WorldState>(
        "/tmp/tbots/world_state", [this](TbotsProto::WorldState input) {
            std::scoped_lock lock(simulator_mutex);
            this->er_force_sim_->setWorldState(input);
            LOG(DEBUG) << "Reconfigured to " << input.DebugString();
        }));

    wrapper_packet_output_.reset(new ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket>(
        "/tmp/tbots/ssl_wrapper_packet"));

    simulation_tick_input_.reset(new ThreadedProtoUnixListener<TbotsProto::SimulatorTick>(
        "/tmp/tbots/simulation_tick", [this](TbotsProto::SimulatorTick input) {
            std::scoped_lock lock(simulator_mutex);

            this->er_force_sim_->stepSimulation(
                Duration::fromMilliseconds(input.milliseconds()));

            for (auto packet : this->er_force_sim_->getSSLWrapperPackets())
            {
                wrapper_packet_output_->sendProto(packet);
            }
        }));

    yellow_primitive_set_input_.reset(
        new ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
            "/tmp/tbots/yellow_primitive_set", [this](TbotsProto::PrimitiveSet input) {
                std::scoped_lock lock(simulator_mutex);
                this->er_force_sim_->setYellowRobotPrimitiveSet(
                    input, std::make_unique<TbotsProto::Vision>(yellow_vision_));
            }));

    blue_primitive_set_input_.reset(
        new ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
            "/tmp/tbots/blue_primitive_set", [this](TbotsProto::PrimitiveSet input) {
                std::scoped_lock lock(simulator_mutex);
                this->er_force_sim_->setBlueRobotPrimitiveSet(
                    input, std::make_unique<TbotsProto::Vision>(blue_vision_));
            }));

    blue_vision_input_.reset(new ThreadedProtoUnixListener<TbotsProto::Vision>(
        "/tmp/tbots/blue_vision", [this](TbotsProto::Vision input) {
            std::scoped_lock lock(simulator_mutex);
            blue_vision_ = input;
        }));

    yellow_vision_input_.reset(new ThreadedProtoUnixListener<TbotsProto::Vision>(
        "/tmp/tbots/yellow_vision", [this](TbotsProto::Vision input) {
            std::scoped_lock lock(simulator_mutex);
            yellow_vision_ = input;
        }));

    er_force_sim_.reset(new ErForceSimulator(
        TbotsProto::FieldType::DIV_B, create2021RobotConstants(),
        create2021WheelConstants(), std::make_shared<const SimulatorConfig>()));
}

StandaloneErForceSimulator::~StandaloneErForceSimulator() {}
