#include "proto/tbots_software_msgs.pb.h"
#include "proto/vision.pb.h"
#include "proto/world.pb.h"
#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/networking/threaded_proto_unix_listener.hpp"
#include "software/networking/threaded_proto_unix_sender.hpp"
#include "software/simulation/er_force_simulator.h"

int main(int argc, char **argv)
{
    auto args               = std::make_shared<StandaloneSimulatorMainCommandLineArgs>();
    bool help_requested     = args->loadFromCommandLineArguments(argc, argv);
    std::string runtime_dir = args->getRuntimeDir()->value();

    LoggerSingleton::initializeLogger(runtime_dir);

    if (!help_requested)
    {
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
         */
        std::shared_ptr<ErForceSimulator> er_force_sim;

        // Setup the field
        if (args->getSslDivision()->value() == "div_a")
        {
            er_force_sim = std::make_shared<ErForceSimulator>(
                TbotsProto::FieldType::DIV_A, create2021RobotConstants(),
                create2021WheelConstants(), std::make_shared<const SimulatorConfig>());
        }
        else
        {
            er_force_sim = std::make_shared<ErForceSimulator>(
                TbotsProto::FieldType::DIV_B, create2021RobotConstants(),
                create2021WheelConstants(), std::make_shared<const SimulatorConfig>());
        }
        std::mutex simulator_mutex;

        // Vision Buffer
        TbotsProto::Vision blue_vision;
        TbotsProto::Vision yellow_vision;

        // Outputs
        // SSL Wrapper Output
        auto ssl_wrapper_output = ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket>(
            runtime_dir + SSL_WRAPPER_PACKET_PATH);

        // Robot Status Outputs
        auto blue_robot_status_output = ThreadedProtoUnixSender<TbotsProto::RobotStatus>(
            runtime_dir + BLUE_ROBOT_STATUS_PATH);
        auto yellow_robot_status_output =
            ThreadedProtoUnixSender<TbotsProto::RobotStatus>(runtime_dir +
                                                             YELLOW_ROBOT_STATUS_PATH);

        // Inputs
        // Word State Input: Configures the ERForceSimulator
        auto world_state_input = ThreadedProtoUnixListener<TbotsProto::WorldState>(
            runtime_dir + WORLD_STATE_PATH, [&](TbotsProto::WorldState input) {
                std::scoped_lock lock(simulator_mutex);
                er_force_sim->setWorldState(input);
            });

        // Vision Input: Buffer vision until we have primitives to tick
        // the simulator with
        auto blue_vision_input = ThreadedProtoUnixListener<TbotsProto::Vision>(
            runtime_dir + BLUE_VISION_PATH, [&](TbotsProto::Vision input) {
                std::scoped_lock lock(simulator_mutex);
                blue_vision = input;
            });
        auto yellow_vision_input = ThreadedProtoUnixListener<TbotsProto::Vision>(
            runtime_dir + YELLOW_VISION_PATH, [&](TbotsProto::Vision input) {
                std::scoped_lock lock(simulator_mutex);
                yellow_vision = input;
            });

        // PrimitiveSet Input: set the primitive set with cached vision
        auto yellow_primitive_set_input =
            ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
                runtime_dir + YELLOW_PRIMITIVE_SET, [&](TbotsProto::PrimitiveSet input) {
                    std::scoped_lock lock(simulator_mutex);
                    er_force_sim->setYellowRobotPrimitiveSet(
                        input, std::make_unique<TbotsProto::Vision>(yellow_vision));
                });

        auto blue_primitive_set_input =
            ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
                runtime_dir + BLUE_PRIMITIVE_SET, [&](TbotsProto::PrimitiveSet input) {
                    std::scoped_lock lock(simulator_mutex);
                    er_force_sim->setBlueRobotPrimitiveSet(
                        input, std::make_unique<TbotsProto::Vision>(blue_vision));
                });

        // Simulator Tick Input
        auto simulator_tick = ThreadedProtoUnixListener<TbotsProto::SimulatorTick>(
            runtime_dir + SIMULATION_TICK_PATH, [&](TbotsProto::SimulatorTick input) {
                std::scoped_lock lock(simulator_mutex);

                // Step the simulation and send back the wrapper packets and
                // the robot status msgs
                er_force_sim->stepSimulation(
                    Duration::fromMilliseconds(input.milliseconds()));

                for (const auto packet : er_force_sim->getSSLWrapperPackets())
                {
                    ssl_wrapper_output.sendProto(packet);
                }

                for (const auto packet : er_force_sim->getBlueRobotStatuses())
                {
                    blue_robot_status_output.sendProto(packet);
                }

                for (const auto packet : er_force_sim->getYellowRobotStatuses())
                {
                    yellow_robot_status_output.sendProto(packet);
                }
            });

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }
}
