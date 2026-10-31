#include <boost/program_options.hpp>
#include <fstream>
#include <queue>

#include "extlibs/er_force_sim/src/protobuf/world.pb.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/sensor_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/vision.pb.h"
#include "proto/world.pb.h"
#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/networking/unix/threaded_proto_unix_listener.hpp"
#include "software/networking/unix/threaded_proto_unix_sender.hpp"
#include "software/simulation/avah_logger.hpp"
#include "software/simulation/er_force_simulator.h"

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

        avah::set_path(args.runtime_dir + "/avah_sim.log");

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
                avah::AVAH_LOG("Received blue vision");
                avah::AVAH_LOG_PROTO(world);
                avah::AVAH_LOG("Received blue prim");
                avah::AVAH_LOG_PROTO(prim);

                er_force_sim->setBlueRobotPrimitiveSet(
                    prim, std::make_unique<TbotsProto::World>(world));
            }

            {
                auto& world = yellow_visions.front();
                auto& prim  = yellow_prims.front();
                avah::AVAH_LOG("Received yellow vision");
                avah::AVAH_LOG_PROTO(world);
                avah::AVAH_LOG("Received yellow prim");
                avah::AVAH_LOG_PROTO(prim);

                er_force_sim->setYellowRobotPrimitiveSet(
                    prim, std::make_unique<TbotsProto::World>(world));
            }

            blue_visions.pop();
            blue_prims.pop();
            yellow_visions.pop();
            yellow_prims.pop();
            avah::AVAH_LOG("Send avah ack");
            avah_ack_output.sendProto(TbotsProto::AvahAck{});
        };

        // Outputs
        // SSL Wrapper Output (used by Thunderscope to visualize the simulation)
        auto common_ssl_wrapper_output =
            ThreadedProtoUnixSender<SSLProto::SSL_WrapperPacket>(runtime_dir +
                                                                 SSL_WRAPPER_PATH);

        // Per-team SensorProto Output
        //
        // Each team gets its vision frame and the robot statuses for that same frame
        // in a single message. Sending them separately means sensor fusion in the
        // full system has to win a race between two sockets to know about a
        // breakbeam trip before it processes the matching vision frame, which makes
        // the resulting World (and therefore the simulation) nondeterministic.
        auto blue_sensor_proto_output =
            ThreadedProtoUnixSender<SensorProto>(runtime_dir + BLUE_SENSOR_PROTO_PATH);
        auto yellow_sensor_proto_output =
            ThreadedProtoUnixSender<SensorProto>(runtime_dir + YELLOW_SENSOR_PROTO_PATH);

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
                avah::AVAH_LOG("GOTSENTWORLDSTATE");
                avah::AVAH_LOG_PROTO(input);
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
                // avah::AVAH_LOG("Received blue world");
                // avah::AVAH_LOG_PROTO(input);
                std::scoped_lock lock(simulator_mutex);
                blue_visions.push(input);
                avah_send_if_both();
            });

        auto yellow_world_input = ThreadedProtoUnixListener<TbotsProto::World>(
            runtime_dir + YELLOW_WORLD_PATH,
            [&](TbotsProto::World input)
            {
                // avah::AVAH_LOG("Received yellow world");
                // avah::AVAH_LOG_PROTO(input);
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
                    // avah::AVAH_LOG("Received yellow primitive set");
                    // avah::AVAH_LOG_PROTO(input);
                    std::scoped_lock lock(simulator_mutex);
                    yellow_prims.push(input);
                    avah_send_if_both();
                });

        auto blue_primitive_set_input =
            ThreadedProtoUnixListener<TbotsProto::PrimitiveSet>(
                runtime_dir + BLUE_PRIMITIVE_SET,
                [&](TbotsProto::PrimitiveSet input)
                {
                    // avah::AVAH_LOG("Received blue primitive set");
                    // avah::AVAH_LOG_PROTO(input);
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

                avah::AVAH_LOG("Received simulator tick");
                avah::AVAH_LOG_PROTO(input);

                avah::AVAH_LOG("BEFORE SIM STEP");
                avah::AVAH_LOG_PROTO(er_force_sim->getSimulatorState());

                // Step the simulation and send back the wrapper packets and
                // the robot status msgs
                er_force_sim->stepSimulation(
                    Duration::fromMilliseconds(input.milliseconds()));

                avah::AVAH_LOG("AFTER SIM STEP");
                avah::AVAH_LOG_PROTO(er_force_sim->getSimulatorState());

                const auto wrapper_packets       = er_force_sim->getSSLWrapperPackets();
                const auto blue_robot_statuses   = er_force_sim->getBlueRobotStatuses();
                const auto yellow_robot_statuses = er_force_sim->getYellowRobotStatuses();

                for (const auto& packet : wrapper_packets)
                {
                    avah::AVAH_LOG("Sending SSL wrapper packets");
                    avah::AVAH_LOG_PROTO(packet);
                    common_ssl_wrapper_output.sendProto(packet);

                    // The robot statuses describe the state of the robots at the
                    // instant this vision frame was captured, so stamp them with the
                    // frame's capture time and hand them over together with the frame
                    auto blue_sensor_proto                      = SensorProto();
                    *blue_sensor_proto.mutable_ssl_vision_msg() = packet;
                    for (auto robot_status : blue_robot_statuses)
                    {
                        robot_status.mutable_time_sent()->set_epoch_timestamp_seconds(
                            packet.detection().t_capture());
                        *blue_sensor_proto.add_robot_status_msgs() = robot_status;
                    }
                    avah::AVAH_LOG("Sending blue sensor proto");
                    avah::AVAH_LOG_PROTO(blue_sensor_proto);
                    blue_sensor_proto_output.sendProto(blue_sensor_proto);

                    auto yellow_sensor_proto                      = SensorProto();
                    *yellow_sensor_proto.mutable_ssl_vision_msg() = packet;
                    for (auto robot_status : yellow_robot_statuses)
                    {
                        robot_status.mutable_time_sent()->set_epoch_timestamp_seconds(
                            packet.detection().t_capture());
                        *yellow_sensor_proto.add_robot_status_msgs() = robot_status;
                    }
                    avah::AVAH_LOG("Sending yellow sensor proto");
                    avah::AVAH_LOG_PROTO(yellow_sensor_proto);
                    yellow_sensor_proto_output.sendProto(yellow_sensor_proto);
                }

                avah::AVAH_LOG("Sending simulator state");
                avah::AVAH_LOG_PROTO(er_force_sim->getSimulatorState());
                simulator_state_output.sendProto(er_force_sim->getSimulatorState());
            });

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }
}
