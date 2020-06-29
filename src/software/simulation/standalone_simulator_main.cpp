#include "shared/constants.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/constants.h"
#include "software/gui/standalone_simulator/standalone_simulator_gui_wrapper.h"
#include "software/logger/logger.h"
#include "software/networking/threaded_proto_multicast_listener.h"
#include "software/networking/threaded_proto_multicast_sender.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/primitive/primitive.h"
#include "software/simulation/threaded_simulator.h"
#include "software/world/field.h"

/**
 * Decodes a PrimitiveMsg into its primitive index and parameters
 *
 * @param msg The message to decode
 *
 * @return The primitive index and parameters for the given PrimitiveMsg
 */
std::pair<unsigned int, primitive_params_t> decodePrimitiveMsg(const PrimitiveMsg& msg)
{
    auto primitive_index = static_cast<unsigned int>(msg.prim_type());

    primitive_params_t params;
    params.params[0] = static_cast<uint16_t>(msg.parameter1());
    params.params[1] = static_cast<uint16_t>(msg.parameter2());
    params.params[2] = static_cast<uint16_t>(msg.parameter3());
    params.params[3] = static_cast<uint16_t>(msg.parameter4());
    params.extra     = static_cast<uint8_t>(msg.extra_bits());
    params.slow      = msg.slow();

    return std::make_pair(primitive_index, params);
}

int main(int argc, char* argv[])
{
    LoggerSingleton::initializeLogger();

    auto network_interface =
        DynamicParameters->getNetworkConfig()->NetworkInterface()->value();
    int yellow_team_channel =
        DynamicParameters->getStandaloneSimulatorConfig()->YellowTeamChannel()->value();
    std::string yellow_team_ip =
        std::string(MULTICAST_CHANNELS[yellow_team_channel]) + "%" + network_interface;
    int blue_team_channel =
        DynamicParameters->getStandaloneSimulatorConfig()->BlueTeamChannel()->value();
    std::string blue_team_ip =
        std::string(MULTICAST_CHANNELS[blue_team_channel]) + "%" + network_interface;

    StandaloneSimulatorGUIWrapper standalone_simulator_gui_wrapper(argc, argv);
    ThreadedSimulator simulator(Field::createSSLDivisionBField());

    ThreadedProtoMulticastSender<SSL_WrapperPacket> wrapper_packet_sender(
        SSL_VISION_DEFAULT_MULTICAST_ADDRESS, SSL_VISION_MULTICAST_PORT);
    simulator.registerOnSSLWrapperPacketReadyCallback(
        [&standalone_simulator_gui_wrapper,
         &wrapper_packet_sender](SSL_WrapperPacket packet) {
            wrapper_packet_sender.sendProto(packet);
            standalone_simulator_gui_wrapper.onValueReceived(packet);
        });

    ThreadedProtoMulticastListener<PrimitiveSetMsg> yellow_team_primitive_listener(
        yellow_team_ip, PRIMITIVE_PORT, [&simulator](PrimitiveSetMsg msg) {
            for (const auto& primitive : msg.robot_primitives())
            {
                RobotId id = primitive.first;
                auto [primitive_index, primitive_params] =
                    decodePrimitiveMsg(primitive.second);
                simulator.setYellowRobotPrimitive(id, primitive_index, primitive_params);
            }
        });
    ThreadedProtoMulticastListener<PrimitiveSetMsg> blue_team_primitive_listener(
        blue_team_ip, PRIMITIVE_PORT, [&simulator](PrimitiveSetMsg msg) {
            for (const auto& primitive : msg.robot_primitives())
            {
                RobotId id = primitive.first;
                auto [primitive_index, primitive_params] =
                    decodePrimitiveMsg(primitive.second);
                simulator.setBlueRobotPrimitive(id, primitive_index, primitive_params);
            }
        });

    simulator.setBallState(BallState(Point(0, 0), Vector(0, 0)));

    RobotState blue_robot_state1(Point(-1, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero());
    RobotState blue_robot_state2(Point(-2, 1), Vector(0, 0), Angle::quarter(),
                                 AngularVelocity::zero());
    std::vector<RobotStateWithId> blue_robot_states = {
        RobotStateWithId{.id = 1, .robot_state = blue_robot_state1},
        RobotStateWithId{.id = 2, .robot_state = blue_robot_state2},
    };
    simulator.addBlueRobots(blue_robot_states);

    RobotState yellow_robot_state1(Point(1, 1.5), Vector(0, 0), Angle::half(),
                                   AngularVelocity::zero());
    RobotState yellow_robot_state2(Point(2.5, -1), Vector(0, 0), Angle::threeQuarter(),
                                   AngularVelocity::zero());
    std::vector<RobotStateWithId> yellow_robot_states = {
        RobotStateWithId{.id = 1, .robot_state = yellow_robot_state1},
        RobotStateWithId{.id = 2, .robot_state = yellow_robot_state2},
    };
    simulator.addYellowRobots(yellow_robot_states);

    simulator.startSimulation();

    // This blocks forever without using the CPU.
    // Wait for the Simulator GUI to shut down before shutting
    // down the rest of the system
    standalone_simulator_gui_wrapper.getTerminationPromise()->get_future().wait();

    simulator.stopSimulation();

    return 0;
}
