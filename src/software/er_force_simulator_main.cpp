#include <boost/program_options.hpp>
#include <memory>

#include "extlibs/er_force_sim/src/protobuf/world.pb.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/world.pb.h"
#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/networking/udp/threaded_proto_udp_sender.hpp"
#include "software/networking/unix/threaded_proto_unix_listener.hpp"
#include "software/networking/unix/threaded_proto_unix_sender.hpp"
#include "software/simulation/er_force_simulator.h"
#include "software/simulation/interface/simulator_team_interface.h"
#include "software/simulation/interface/udp_ssl_interface.h"
#include "software/simulation/interface/unix_primitive_interface.h"

const uint16_t BLUE_UDP_PORT   = 10301;
const uint16_t YELLOW_UDP_PORT = 10302;

int main(int argc, char** argv)
{
    struct CommandLineArgs
    {
        bool help                        = false;
        std::string runtime_dir          = "/tmp/tbots";
        std::string division             = "div_b";
        bool enable_realism              = false;  // realism flag
        bool blue_udp_ssl                = false;
        bool yellow_udp_ssl              = false;
        uint16_t blue_udp_port           = BLUE_UDP_PORT;
        uint16_t yellow_udp_port         = YELLOW_UDP_PORT;
        uint16_t ssl_vision_port         = SSL_VISION_PORT;
        std::string ssl_vision_address   = SSL_VISION_ADDRESS;
        std::string ssl_vision_interface = "lo";
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
    desc.add_options()("blue_udp_ssl",
                       boost::program_options::bool_switch(&args.blue_udp_ssl),
                       "use udp ssl instead of unix sockets to control blue team");
    desc.add_options()("yellow_udp_ssl",
                       boost::program_options::bool_switch(&args.yellow_udp_ssl),
                       "use udp ssl instead of unix sockets to control yellow team");

    desc.add_options()("blue_udp_port",
                       boost::program_options::value<uint16_t>(&args.blue_udp_port),
                       "Blue SSL robot control UDP port (default 10301)");
    desc.add_options()("yellow_udp_port",
                       boost::program_options::value<uint16_t>(&args.yellow_udp_port),
                       "Yellow SSL robot control UDP port (default 10302)");
    desc.add_options()("ssl_vision_port",
                       boost::program_options::value<uint16_t>(&args.ssl_vision_port),
                       "SSL-Vision destination UDP port (default 10006)");
    desc.add_options()(
        "ssl_vision_address",
        boost::program_options::value<std::string>(&args.ssl_vision_address),
        "SSL-Vision destination address (default 224.5.23.2)");
    desc.add_options()(
        "ssl_vision_interface",
        boost::program_options::value<std::string>(&args.ssl_vision_interface),
        "SSL-Vision multicast interface (default lo)");

    boost::program_options::variables_map vm;
    boost::program_options::store(parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (args.help)
    {
        std::cout << desc << std::endl;
    }
    else
    {
        if ((args.blue_udp_ssl && args.blue_udp_port == 0) ||
            (args.yellow_udp_ssl && args.yellow_udp_port == 0) ||
            ((args.blue_udp_ssl || args.yellow_udp_ssl) && args.ssl_vision_port == 0) ||
            (args.blue_udp_ssl && args.yellow_udp_ssl &&
             args.blue_udp_port == args.yellow_udp_port))
        {
            std::cerr
                << "Teams communicating to simulator over UDP must have different, non-zero ports.\n";
            return 1;
        }
        std::string runtime_dir = args.runtime_dir;
        LoggerSingleton::initializeLogger(runtime_dir, nullptr);

        /**
         * Creates a ER force simulator and sets up the appropriate
         * communication channels (unix senders/listeners or udp sockets). Below
         * is an example where yellow speaks over unix socket and blue speaks over
         * UDP (with ssl instead of primitives)
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
         *   Blue Robot Controls  │                            │
         *   ─────────────────────►  ┌──────────────────────┐  │ Blue Robot Feedback
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
                realism_config, true, args.blue_udp_ssl, args.yellow_udp_ssl);
        }
        else
        {
            er_force_sim = std::make_shared<ErForceSimulator>(
                TbotsProto::FieldType::DIV_B, robot_constants::createRobotConstants(),
                realism_config, true, args.blue_udp_ssl, args.yellow_udp_ssl);
        }

        std::mutex simulator_mutex;

        std::unique_ptr<SimulatorTeamInterface> blue_interface;
        std::unique_ptr<SimulatorTeamInterface> yellow_interface;

        {
            std::scoped_lock lock(simulator_mutex);

            // start blue team interface
            if (args.blue_udp_ssl)
                blue_interface = std::make_unique<UdpSslInterface>(
                    er_force_sim, simulator_mutex, true, args.blue_udp_port);
            else
                blue_interface = std::make_unique<UnixPrimitiveInterface>(
                    er_force_sim, simulator_mutex, true,
                    runtime_dir + BLUE_SSL_WRAPPER_PATH,
                    runtime_dir + BLUE_ROBOT_STATUS_PATH, runtime_dir + BLUE_WORLD_PATH,
                    runtime_dir + BLUE_PRIMITIVE_SET);

            // start yellow team interface
            if (args.yellow_udp_ssl)
                yellow_interface = std::make_unique<UdpSslInterface>(
                    er_force_sim, simulator_mutex, false, args.yellow_udp_port);
            else
                yellow_interface = std::make_unique<UnixPrimitiveInterface>(
                    er_force_sim, simulator_mutex, false,
                    runtime_dir + YELLOW_SSL_WRAPPER_PATH,
                    runtime_dir + YELLOW_ROBOT_STATUS_PATH,
                    runtime_dir + YELLOW_WORLD_PATH, runtime_dir + YELLOW_PRIMITIVE_SET);

            blue_interface->startListening();
            yellow_interface->startListening();
        }

        auto common_ssl_wrapper_output =
            ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket>(runtime_dir +
                                                                 SSL_WRAPPER_PATH);


        // Simulator State as World State Output
        auto simulator_state_output = ThreadedProtoUnixSender<world::SimulatorState>(
            runtime_dir + SIMULATOR_STATE_PATH);


        // World State Received Trigger as Simulator Output
        auto world_state_received_trigger =
            ThreadedProtoUnixSender<TbotsProto::WorldStateReceivedTrigger>(
                runtime_dir + WORLD_STATE_RECEIVED_TRIGGER_PATH);

        bool has_sent_world_state_trigger = false;

        std::unique_ptr<ThreadedProtoUdpSender<SSLProto::SSL_WrapperPacket>>
            udp_vision_output;
        if (args.blue_udp_ssl || args.yellow_udp_ssl)
        {
            const auto address = boost::asio::ip::make_address(args.ssl_vision_address);
            udp_vision_output =
                std::make_unique<ThreadedProtoUdpSender<SSLProto::SSL_WrapperPacket>>(
                    args.ssl_vision_address, args.ssl_vision_port,
                    args.ssl_vision_interface, address.is_multicast());
        }

        // Inputs
        // World State Input: Configures the ERForceSimulator
        auto world_state_input = ThreadedProtoUnixListener<TbotsProto::WorldState>(
            runtime_dir + WORLD_STATE_PATH,
            [&](TbotsProto::WorldState input)
            {
                std::scoped_lock lock(simulator_mutex);
                er_force_sim->setWorldState(input);

                if (!has_sent_world_state_trigger)
                {
                    auto world_state_received_trigger_msg =
                        *createWorldStateReceivedTrigger();
                    world_state_received_trigger.sendProto(
                        world_state_received_trigger_msg);
                    has_sent_world_state_trigger = true;
                }
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

                // only perform once, has randomness and modifies sim state :/
                auto sslPackets = er_force_sim->getSSLWrapperPackets();

                blue_interface->postSimulatorTick(sslPackets);
                yellow_interface->postSimulatorTick(sslPackets);

                for (const auto& packet : sslPackets)
                {
                    common_ssl_wrapper_output.sendProto(packet);
                    if (udp_vision_output)
                    {
                        udp_vision_output->sendProto(packet);
                    }
                }

                simulator_state_output.sendProto(er_force_sim->getSimulatorState());
            });

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }
}
