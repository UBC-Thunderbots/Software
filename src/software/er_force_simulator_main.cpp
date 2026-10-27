#include <google/protobuf/text_format.h>

#include <boost/program_options.hpp>
#include <fstream>
#include <queue>

#include "extlibs/er_force_sim/src/protobuf/world.pb.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/vision.pb.h"
#include "proto/world.pb.h"
#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/networking/unix/threaded_proto_unix_listener.hpp"
#include "software/networking/unix/threaded_proto_unix_sender.hpp"
#include "software/simulation/er_force_simulator.h"

namespace
{

std::string path;
static int i{0};

[[maybe_unused]] void AVAH_LOG(auto x)
{
    std::ofstream out{path, std::ios::app};
    out << i++ << ": " << x << "\n\n";
    out.close();
}

[[maybe_unused]] void AVAH_LOG_PROTO(auto x)
{
    std::string text;
    if (google::protobuf::TextFormat::PrintToString(x, &text))
    {
        AVAH_LOG(text);
    }
    else
    {
        LOG(FATAL) << "COULDNT LOG PROTO";
    }
}

}  // namespace

int main(int argc, char** argv)
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
        LoggerSingleton::initializeLogger(runtime_dir, nullptr);

        path = args.runtime_dir + "/avah_sim.log";

        {
            std::ofstream out{path};
            out.close();
        }

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
                TbotsProto::FieldType::DIV_A, robot_constants::createRobotConstants(),
                realism_config);
        }
        else
        {
            er_force_sim = std::make_shared<ErForceSimulator>(
                TbotsProto::FieldType::DIV_B, robot_constants::createRobotConstants(),
                realism_config);
        }

        std::mutex simulator_mutex;

        // World Buffer
        std::queue<TbotsProto::World> blue_visions;
        std::queue<TbotsProto::World> yellow_visions;

        std::queue<TbotsProto::PrimitiveSet> blue_prims;
        std::queue<TbotsProto::PrimitiveSet> yellow_prims;

        auto avah_ack_output =
            ThreadedProtoUnixSender<TbotsProto::AvahAck>(runtime_dir + "/avah_ack");

        auto avah_send_if_both = [&]()
        {
            if (blue_visions.empty() || blue_prims.empty() || yellow_visions.empty() ||
                yellow_prims.empty())
            {
                return;
            }

            {
                auto& world = blue_visions.front();
                auto& prim  = blue_prims.front();
                AVAH_LOG("Received blue vision");
                AVAH_LOG_PROTO(world);
                AVAH_LOG("Received blue prim");
                AVAH_LOG_PROTO(prim);

                er_force_sim->setBlueRobotPrimitiveSet(
                    prim, std::make_unique<TbotsProto::World>(world));
            }

            {
                auto& world = yellow_visions.front();
                auto& prim  = yellow_prims.front();
                AVAH_LOG("Received yellow vision");
                AVAH_LOG_PROTO(world);
                AVAH_LOG("Received yellow prim");
                AVAH_LOG_PROTO(prim);

                er_force_sim->setYellowRobotPrimitiveSet(
                    prim, std::make_unique<TbotsProto::World>(world));
            }

            blue_visions.pop();
            blue_prims.pop();
            yellow_visions.pop();
            yellow_prims.pop();
            AVAH_LOG("Send avah ack");
            avah_ack_output.sendProto(TbotsProto::AvahAck{});
        };

        // Outputs
        // SSL Wrapper Output
        auto blue_ssl_wrapper_output =
            ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket>(runtime_dir +
                                                                 BLUE_SSL_WRAPPER_PATH);
        auto yellow_ssl_wrapper_output =
            ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket>(runtime_dir +
                                                                 YELLOW_SSL_WRAPPER_PATH);
        auto common_ssl_wrapper_output =
            ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket>(runtime_dir +
                                                                 SSL_WRAPPER_PATH);

        // Robot Status Outputs
        auto blue_robot_status_output = ThreadedProtoUnixSender<TbotsProto::RobotStatus>(
            runtime_dir + BLUE_ROBOT_STATUS_PATH);
        auto yellow_robot_status_output =
            ThreadedProtoUnixSender<TbotsProto::RobotStatus>(runtime_dir +
                                                             YELLOW_ROBOT_STATUS_PATH);

        // Simulator State as World State Output
        auto simulator_state_output = ThreadedProtoUnixSender<world::SimulatorState>(
            runtime_dir + SIMULATOR_STATE_PATH);


        // World State Received Trigger as Simulator Output
        auto world_state_received_trigger =
            ThreadedProtoUnixSender<TbotsProto::WorldStateReceivedTrigger>(
                runtime_dir + WORLD_STATE_RECEIVED_TRIGGER_PATH);

        bool has_sent_world_state_trigger = false;

        // Inputs
        // World State Input: Configures the ERForceSimulator
        auto world_state_input = ThreadedProtoUnixListener<TbotsProto::WorldState>(
            runtime_dir + WORLD_STATE_PATH,
            [&](TbotsProto::WorldState input)
            {
                AVAH_LOG("Received world state");
                AVAH_LOG_PROTO(input);
                std::scoped_lock lock(simulator_mutex);
                er_force_sim->setWorldState(input);

                if (!has_sent_world_state_trigger)
                {
                    auto world_state_received_trigger_msg =
                        *createWorldStateReceivedTrigger();
                    world_state_received_trigger.sendProto(
                        world_state_received_trigger_msg);
                    has_sent_world_state_trigger = true;
                    LOG(INFO) << "Received world state";
                }
            });

        // World Input: Buffer vision until we have primitives to tick
        // the simulator with
        auto blue_world_input = ThreadedProtoUnixListener<TbotsProto::World>(
            runtime_dir + BLUE_WORLD_PATH,
            [&](TbotsProto::World input)
            {
                // AVAH_LOG("Received blue world");
                // AVAH_LOG_PROTO(input);
                std::scoped_lock lock(simulator_mutex);
                blue_visions.push(input);
                avah_send_if_both();
            });

        auto yellow_world_input = ThreadedProtoUnixListener<TbotsProto::World>(
            runtime_dir + YELLOW_WORLD_PATH,
            [&](TbotsProto::World input)
            {
                // AVAH_LOG("Received yellow world");
                // AVAH_LOG_PROTO(input);
                std::scoped_lock lock(simulator_mutex);
                yellow_visions.push(input);
                avah_send_if_both();
            });

        // PrimitiveSet Input: set the primitive set with cached vision
        auto yellow_primitive_set_input =
            ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
                runtime_dir + YELLOW_PRIMITIVE_SET,
                [&](TbotsProto::PrimitiveSet input)
                {
                    // AVAH_LOG("Received yellow primitive set");
                    // AVAH_LOG_PROTO(input);
                    std::scoped_lock lock(simulator_mutex);
                    yellow_prims.push(input);
                    avah_send_if_both();
                });

        auto blue_primitive_set_input =
            ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
                runtime_dir + BLUE_PRIMITIVE_SET,
                [&](TbotsProto::PrimitiveSet input)
                {
                    // AVAH_LOG("Received blue primitive set");
                    // AVAH_LOG_PROTO(input);
                    std::scoped_lock lock(simulator_mutex);
                    blue_prims.push(input);
                    avah_send_if_both();
                });

        // Simulator Tick Input
        auto simulator_tick = ThreadedProtoUnixListener<TbotsProto::SimulatorTick>(
            runtime_dir + SIMULATION_TICK_PATH,
            [&](TbotsProto::SimulatorTick input)
            {
                std::scoped_lock lock(simulator_mutex);

                // Step the simulation and send back the wrapper packets and
                // the robot status msgs
                er_force_sim->stepSimulation(
                    Duration::fromMilliseconds(input.milliseconds()));

                AVAH_LOG("Received simulator tick");
                AVAH_LOG_PROTO(input);

                for (const auto& packet : er_force_sim->getSSLWrapperPackets())
                {
                    AVAH_LOG("Sending SSL wrapper packets");
                    AVAH_LOG_PROTO(packet);
                    blue_ssl_wrapper_output.sendProto(packet);
                    yellow_ssl_wrapper_output.sendProto(packet);
                    common_ssl_wrapper_output.sendProto(packet);
                }

                for (const auto& packet : er_force_sim->getBlueRobotStatuses())
                {
                    AVAH_LOG("Sending blue robot statuses");
                    AVAH_LOG_PROTO(packet);
                    blue_robot_status_output.sendProto(packet);
                }

                for (const auto& packet : er_force_sim->getYellowRobotStatuses())
                {
                    AVAH_LOG("Sending yellow robot statuses");
                    AVAH_LOG_PROTO(packet);
                    yellow_robot_status_output.sendProto(packet);
                }

                AVAH_LOG("Sending simulator state");
                AVAH_LOG_PROTO(er_force_sim->getSimulatorState());
                simulator_state_output.sendProto(er_force_sim->getSimulatorState());
            });

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }
}
