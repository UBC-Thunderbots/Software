#include <boost/program_options.hpp>

#include "extlibs/er_force_sim/src/protobuf/world.pb.h"
#include "proto/message_translation/tbots_protobuf.h"
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
    struct CommandLineArgs
    {
        bool help               = false;
        std::string runtime_dir = "/tmp/tbots";
        std::string division    = "div_b";
        bool enable_realism     = false;  // realism flag
    };

    CommandLineArgs args;
    boost::program_options::options_description desc{"Options"};

    desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                       "Help screen");
    desc.add_options()("runtime_dir",
                       boost::program_options::value<std::string>(&args.runtime_dir),
                       "The directory to output logs and setup unix sockets.");
    desc.add_options()("division",
                       boost::program_options::value<std::string>(&args.division),
                       "div_a or div_b");
    desc.add_options()("enable_realism",
                       boost::program_options::bool_switch(&args.enable_realism),
                       "realism simulator");  // install terminal flag

    boost::program_options::variables_map vm;
    boost::program_options::store(parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (args.help)
    {
        std::cout << desc << std::endl;
    }
    else
    {
        std::string runtime_dir = args.runtime_dir;
        LoggerSingleton::initializeLogger(runtime_dir);

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
         *   Blue World           │  │                      │  │
         *   ─────────────────────►  │                      │  │
         *   Yellow World         │  └──────────────────────┘  │
         *                        └────────────────────────────┘
         */
        std::shared_ptr<ErForceSimulator> er_force_sim;
        std::unique_ptr<RealismConfigErForce> realism_config;

        if (args.enable_realism)
        {
            realism_config = ErForceSimulator::createRealisticRealismConfig();
        }
        else
        {
            realism_config = ErForceSimulator::createDefaultRealismConfig();
        }

        if (args.division == "div_a")
        {
            er_force_sim = std::make_shared<ErForceSimulator>(
                TbotsProto::FieldType::DIV_A, create2021RobotConstants(), realism_config);
        }
        else
        {
            er_force_sim = std::make_shared<ErForceSimulator>(
                TbotsProto::FieldType::DIV_B, create2021RobotConstants(), realism_config);
        }

        std::mutex simulator_mutex;

        // World Buffer
        TbotsProto::World blue_vision;
        TbotsProto::World yellow_vision;

        // Outputs
        // SSL Wrapper Output
        auto blue_ssl_wrapper_output =
            ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket>(runtime_dir +
                                                                 BLUE_SSL_WRAPPER_PATH);
        auto yellow_ssl_wrapper_output =
            ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket>(runtime_dir +
                                                                 YELLOW_SSL_WRAPPER_PATH);

        // Robot Status Outputs
        auto blue_robot_status_output = ThreadedProtoUnixSender<TbotsProto::RobotStatus>(
            runtime_dir + BLUE_ROBOT_STATUS_PATH);
        auto yellow_robot_status_output =
            ThreadedProtoUnixSender<TbotsProto::RobotStatus>(runtime_dir +
                                                             YELLOW_ROBOT_STATUS_PATH);

        // Simulator State as World State Output
        auto simulator_state_output = ThreadedProtoUnixSender<world::SimulatorState>(
            runtime_dir + SIMULATOR_STATE_PATH);

        // Simulation Started Trigger as Simulator Output
        auto simulation_started_trigger =
            ThreadedProtoUnixSender<TbotsProto::SimulationStartedTrigger>(
                runtime_dir + SIMULATION_STARTED_TRIGGER_PATH);

        // Inputs
        // World State Input: Configures the ERForceSimulator
        auto world_state_input = ThreadedProtoUnixListener<TbotsProto::WorldState>(
            runtime_dir + WORLD_STATE_PATH, [&](TbotsProto::WorldState input) {
                std::scoped_lock lock(simulator_mutex);
                er_force_sim->setWorldState(input);

                auto sim_started_trigger_msg = *createSimulationStartedTrigger(true);
                if (!er_force_sim->has_sent_sim_start_trigger)
                {
                    simulation_started_trigger.sendProto(sim_started_trigger_msg);
                    er_force_sim->has_sent_sim_start_trigger = true;
                }
            });

        // World Input: Buffer vision until we have primitives to tick
        // the simulator with
        auto blue_world_input = ThreadedProtoUnixListener<TbotsProto::World>(
            runtime_dir + BLUE_WORLD_PATH, [&](TbotsProto::World input) {
                std::scoped_lock lock(simulator_mutex);
                blue_vision = input;
            });

        auto yellow_world_input = ThreadedProtoUnixListener<TbotsProto::World>(
            runtime_dir + YELLOW_WORLD_PATH, [&](TbotsProto::World input) {
                std::scoped_lock lock(simulator_mutex);
                yellow_vision = input;
            });

        // PrimitiveSet Input: set the primitive set with cached vision
        auto yellow_primitive_set_input =
            ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
                runtime_dir + YELLOW_PRIMITIVE_SET, [&](TbotsProto::PrimitiveSet input) {
                    std::scoped_lock lock(simulator_mutex);
                    er_force_sim->setYellowRobotPrimitiveSet(
                        input, std::make_unique<TbotsProto::World>(yellow_vision));
                });

        auto blue_primitive_set_input =
            ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
                runtime_dir + BLUE_PRIMITIVE_SET, [&](TbotsProto::PrimitiveSet input) {
                    std::scoped_lock lock(simulator_mutex);
                    er_force_sim->setBlueRobotPrimitiveSet(
                        input, std::make_unique<TbotsProto::World>(blue_vision));
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
                    blue_ssl_wrapper_output.sendProto(packet);
                    yellow_ssl_wrapper_output.sendProto(packet);
                }

                for (const auto packet : er_force_sim->getBlueRobotStatuses())
                {
                    blue_robot_status_output.sendProto(packet);
                }

                for (const auto packet : er_force_sim->getYellowRobotStatuses())
                {
                    yellow_robot_status_output.sendProto(packet);
                }

                simulator_state_output.sendProto(er_force_sim->getSimulatorState());
            });

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }
}
