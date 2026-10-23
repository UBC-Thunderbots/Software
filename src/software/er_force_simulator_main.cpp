#include <boost/program_options.hpp>
#include <cmath>

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

// CSV file that the filtered robot state is logged to, alongside the ground truth
// robot state from the simulator, for evaluating the robot filter
static const std::string ROBOT_FILTER_CSV_FILE_NAME = "realistic_robot_filter_v1.csv";

// Wraps an angle to [-pi, pi] so detected and actual orientations are directly
// comparable (e.g. -3.1 rad and 3.1 rad aren't reported as a ~6.2 rad error)
static double wrapAngle(double radians)
{
    return std::atan2(std::sin(radians), std::cos(radians));
}

/**
 * Logs one CSV row per robot of the given team, containing the robot state detected
 * by the robot filter (taken from the team's World) next to the ground truth robot
 * state from the simulator. Robots that haven't been detected yet are skipped.
 *
 * @param team "blue" or "yellow"
 * @param timestamp_s Seconds since the first vision message was received
 * @param actual_robots Ground truth robots of this team from the simulator
 * @param detected_world The World (post robot filter) received from this team's AI
 */
static void logRobotStates(
    const std::string& team, double timestamp_s,
    const google::protobuf::RepeatedPtrField<world::SimRobot>& actual_robots,
    const TbotsProto::World& detected_world)
{
    for (const auto& actual : actual_robots)
    {
        // Find the detected robot with the same id
        const TbotsProto::Robot* detected_robot = nullptr;
        for (const auto& robot : detected_world.friendly_team().team_robots())
        {
            if (robot.id() == actual.id())
            {
                detected_robot = &robot;
                break;
            }
        }

        if (detected_robot == nullptr)
        {
            continue;
        }

        const auto& detected = detected_robot->current_state();
        LOG(CSV, ROBOT_FILTER_CSV_FILE_NAME)
            << timestamp_s << "," << team << "," << actual.id() << ","
            << detected.global_position().x_meters() << ","
            << detected.global_position().y_meters() << ","
            << detected.global_velocity().x_component_meters() << ","
            << detected.global_velocity().y_component_meters() << ","
            << wrapAngle(detected.global_orientation().radians()) << ","
            << detected.global_angular_velocity().radians_per_second() << ","
            << actual.p_x() << "," << actual.p_y() << "," << actual.v_x() << ","
            << actual.v_y() << "," << wrapAngle(actual.angle()) << "," << actual.r_z()
            << "\n";
    }
}

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
        LOG(CSV, ROBOT_FILTER_CSV_FILE_NAME)
            << "timestamp_s,team,robot_id,"
               "detected_x,detected_y,detected_vel_x,detected_vel_y,"
               "detected_orientation,detected_angular_vel,"
               "actual_x,actual_y,actual_vel_x,actual_vel_y,"
               "actual_orientation,actual_angular_vel\n";

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
         *   WorldState           │            Main            │
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
        TbotsProto::World blue_vision;
        TbotsProto::World yellow_vision;

        // Timestamp of the first vision message received, so that logged timestamps
        // start at 0
        double start_timestamp_s = 0.0;

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

                auto simulator_state = er_force_sim->getSimulatorState();

                // Each team's detected robot state is timestamped by when that
                // team's vision (World) was sent. A timestamp of 0 means no vision
                // has been received from that team yet.
                double blue_timestamp_s =
                    blue_vision.time_sent().epoch_timestamp_seconds();
                double yellow_timestamp_s =
                    yellow_vision.time_sent().epoch_timestamp_seconds();

                if (start_timestamp_s == 0.0)
                {
                    start_timestamp_s =
                        blue_timestamp_s != 0.0 ? blue_timestamp_s : yellow_timestamp_s;
                }

                if (blue_timestamp_s != 0.0)
                {
                    logRobotStates("blue", blue_timestamp_s - start_timestamp_s,
                                   simulator_state.blue_robots(), blue_vision);
                }

                if (yellow_timestamp_s != 0.0)
                {
                    logRobotStates("yellow", yellow_timestamp_s - start_timestamp_s,
                                   simulator_state.yellow_robots(), yellow_vision);
                }

                simulator_state_output.sendProto(simulator_state);
            });

        // This blocks forever without using the CPU
        std::promise<void>().get_future().wait();
    }
}
