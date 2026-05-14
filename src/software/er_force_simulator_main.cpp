#include <boost/program_options.hpp>

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
        LOG(CSV, "realism_kalman_filter_v2.csv") << "timestamp_s,fused_x,fused_y,fused_vel_x, fused_vel_y, truth_x,truth_y, true_vel_x, true_vel_y,is_occluded\n";

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
        double start_timestamp_s          = 0.0;

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
                }

                for (const auto packet : er_force_sim->getBlueRobotStatuses())
                {
                    blue_robot_status_output.sendProto(packet);
                }

                for (const auto packet : er_force_sim->getYellowRobotStatuses())
                {
                    yellow_robot_status_output.sendProto(packet);
                }

                auto sim_state = er_force_sim->getSimulatorState();
                double current_ts = yellow_vision.time_sent().epoch_timestamp_seconds();
                if (start_timestamp_s == 0.0)
                {
                    start_timestamp_s = current_ts;
                }
                LOG(CSV, "realism_kalman_filter_v2.csv")
                    << (current_ts - start_timestamp_s) << ","
                    << yellow_vision.ball().current_state().global_position().x_meters()
                    << ","
                    << yellow_vision.ball().current_state().global_position().y_meters()
                    << ","
                    << yellow_vision.ball().current_state().global_velocity().x_component_meters()
                    << ","
                    << yellow_vision.ball().current_state().global_velocity().y_component_meters()
                    << "," << sim_state.ball().p_x() << "," << sim_state.ball().p_y()
                    << "," << sim_state.ball().v_x() << "," << sim_state.ball().v_y()
                    << "," << !er_force_sim->isBallVisible()
                    << "\n";
                simulator_state_output.sendProto(sim_state);
            });

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }
}
