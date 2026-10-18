#include <boost/program_options.hpp>

#include "extlibs/er_force_sim/src/protobuf/world.pb.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/vision.pb.h"
#include "proto/world.pb.h"
#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/networking/udp/threaded_proto_udp_bidirectional.hpp"
#include "software/networking/udp/threaded_proto_udp_sender.hpp"
#include "software/networking/unix/threaded_proto_unix_listener.hpp"
#include "software/networking/unix/threaded_proto_unix_sender.hpp"
#include "software/simulation/er_force_simulator.h"

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
                << "Enabled UDP control ports must be nonzero and distinct; vision port must be nonzero\n";
            return 1;
        }
        std::string runtime_dir = args.runtime_dir;
        LoggerSingleton::initializeLogger(runtime_dir, nullptr);

        /**
         * Creates a ER force simulator and sets up the appropriate
         * communication channels (unix senders/listeners). All inputs (left) and
         * outputs (right) shown below are over unix sockets. Optional SSL UDP inputs
         * bypass primitive execution; vision is also published over UDP when enabled.
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
                realism_config, true, args.blue_udp_ssl, args.yellow_udp_ssl);
        }
        else
        {
            er_force_sim = std::make_shared<ErForceSimulator>(
                TbotsProto::FieldType::DIV_B, robot_constants::createRobotConstants(),
                realism_config, true, args.blue_udp_ssl, args.yellow_udp_ssl);
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

        using SSLRobotEndpoint =
            ThreadedProtoUdpBidirectional<SSLSimulationProto::RobotControl,
                                          SSLSimulationProto::RobotControlResponse>;
        std::unique_ptr<SSLRobotEndpoint> blue_ssl_input;
        std::unique_ptr<SSLRobotEndpoint> yellow_ssl_input;
        auto handle_ssl_control = [&](bool is_blue,
                                      const SSLSimulationProto::RobotControl& command,
                                      const boost::asio::ip::udp::endpoint& sender)
        {
            std::scoped_lock lock(simulator_mutex);
            const auto response = is_blue ? er_force_sim->sendBlueSSLControl(command)
                                          : er_force_sim->sendYellowSSLControl(command);
            auto& endpoint      = is_blue ? blue_ssl_input : yellow_ssl_input;
            try
            {
                endpoint->sendTo(response, sender);
            }
            catch (const boost::system::system_error& error)
            {
                LOG(WARNING) << "Failed to send SSL robot feedback: " << error.what();
            }
        };
        {
            // Hold the callback lock until each threaded endpoint has been assigned.
            std::scoped_lock lock(simulator_mutex);
            if (args.blue_udp_ssl)
            {
                blue_ssl_input = std::make_unique<SSLRobotEndpoint>(
                    args.blue_udp_port,
                    [handle_ssl_control](const auto& command, const auto& sender)
                    { handle_ssl_control(true, command, sender); });
            }
            if (args.yellow_udp_ssl)
            {
                yellow_ssl_input = std::make_unique<SSLRobotEndpoint>(
                    args.yellow_udp_port,
                    [handle_ssl_control](const auto& command, const auto& sender)
                    { handle_ssl_control(false, command, sender); });
            }
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

        // World Input: Buffer vision until we have primitives to tick
        // the simulator with
        auto blue_world_input = ThreadedProtoUnixListener<TbotsProto::World>(
            runtime_dir + BLUE_WORLD_PATH,
            [&](TbotsProto::World input)
            {
                std::scoped_lock lock(simulator_mutex);
                blue_vision = input;
            });

        auto yellow_world_input = ThreadedProtoUnixListener<TbotsProto::World>(
            runtime_dir + YELLOW_WORLD_PATH,
            [&](TbotsProto::World input)
            {
                std::scoped_lock lock(simulator_mutex);
                yellow_vision = input;
            });

        // PrimitiveSet Input: set the primitive set with cached vision
        auto yellow_primitive_set_input =
            ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
                runtime_dir + YELLOW_PRIMITIVE_SET,
                [&](TbotsProto::PrimitiveSet input)
                {
                    std::scoped_lock lock(simulator_mutex);
                    er_force_sim->setYellowRobotPrimitiveSet(
                        input, std::make_unique<TbotsProto::World>(yellow_vision));
                });

        auto blue_primitive_set_input =
            ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
                runtime_dir + BLUE_PRIMITIVE_SET,
                [&](TbotsProto::PrimitiveSet input)
                {
                    std::scoped_lock lock(simulator_mutex);
                    er_force_sim->setBlueRobotPrimitiveSet(
                        input, std::make_unique<TbotsProto::World>(blue_vision));
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

                for (const auto packet : er_force_sim->getSSLWrapperPackets())
                {
                    blue_ssl_wrapper_output.sendProto(packet);
                    yellow_ssl_wrapper_output.sendProto(packet);
                    common_ssl_wrapper_output.sendProto(packet);
                    if (udp_vision_output)
                    {
                        udp_vision_output->sendProto(packet);
                    }
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
