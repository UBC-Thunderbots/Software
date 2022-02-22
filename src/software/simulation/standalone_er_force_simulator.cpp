#include "software/simulation/standalone_er_force_simulator.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/time/duration.h"

StandaloneErForceSimulator::StandaloneErForceSimulator()
{
    // Create an Er Force Simulator
    er_force_sim_.reset(new ErForceSimulator(
        TbotsProto::FieldType::DIV_B, create2021RobotConstants(),
        create2021WheelConstants(), std::make_shared<const SimulatorConfig>()));

    // Setup outputs
    wrapper_packet_output_.reset(new ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket>(
        BASE_PATH + SSL_WRAPPER_PACKET_PATH));

    blue_robot_status_output_.reset(new ThreadedProtoUnixSender<TbotsProto::RobotStatus>(
        BASE_PATH + BLUE_ROBOT_STATUS_PATH));

    yellow_robot_status_output_.reset(
        new ThreadedProtoUnixSender<TbotsProto::RobotStatus>(BASE_PATH +
                                                             YELLOW_ROBOT_STATUS_PATH));

    // Setup inputs
    world_state_input_.reset(new ThreadedProtoUnixListener<TbotsProto::WorldState>(
        BASE_PATH + WORLD_STATE_PATH, [this](TbotsProto::WorldState input) {
            std::scoped_lock lock(simulator_mutex);
            this->er_force_sim_->setWorldState(input);
            LOG(DEBUG) << input.DebugString();
        }));

    simulation_tick_input_.reset(new ThreadedProtoUnixListener<TbotsProto::SimulatorTick>(
        BASE_PATH + SIMULATION_TICK_PATH, [this](TbotsProto::SimulatorTick input) {
            std::scoped_lock lock(simulator_mutex);

            // Step the simulation and send back the wrapper packets and
            // the robot status msgs
            this->er_force_sim_->stepSimulation(
                Duration::fromMilliseconds(input.milliseconds()));

            for (const auto packet : this->er_force_sim_->getSSLWrapperPackets())
            {
                wrapper_packet_output_->sendProto(packet);
            }

            for (const auto packet : this->er_force_sim_->getBlueRobotStatuses())
            {
                blue_robot_status_output_->sendProto(packet);
            }

            for (const auto packet : this->er_force_sim_->getYellowRobotStatuses())
            {
                yellow_robot_status_output_->sendProto(packet);
            }
        }));

    yellow_primitive_set_input_.reset(
        new ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
            BASE_PATH + YELLOW_PRIMITIVE_SET, [this](TbotsProto::PrimitiveSet input) {
                std::scoped_lock lock(simulator_mutex);
                this->er_force_sim_->setYellowRobotPrimitiveSet(
                    input, std::make_unique<TbotsProto::Vision>(yellow_vision_));
            }));

    blue_primitive_set_input_.reset(
        new ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
            BASE_PATH + BLUE_PRIMITIVE_SET, [this](TbotsProto::PrimitiveSet input) {
                std::scoped_lock lock(simulator_mutex);
                this->er_force_sim_->setBlueRobotPrimitiveSet(
                    input, std::make_unique<TbotsProto::Vision>(blue_vision_));
            }));

    // Just buffer the vision msgs until we have a new primitive to send to the simulator
    blue_vision_input_.reset(new ThreadedProtoUnixListener<TbotsProto::Vision>(
        BASE_PATH + BLUE_VISION_PATH, [this](TbotsProto::Vision input) {
            std::scoped_lock lock(simulator_mutex);
            blue_vision_ = input;
        }));

    yellow_vision_input_.reset(new ThreadedProtoUnixListener<TbotsProto::Vision>(
        BASE_PATH + YELLOW_VISION_PATH, [this](TbotsProto::Vision input) {
            std::scoped_lock lock(simulator_mutex);
            yellow_vision_ = input;
        }));
}

StandaloneErForceSimulator::~StandaloneErForceSimulator() {}
