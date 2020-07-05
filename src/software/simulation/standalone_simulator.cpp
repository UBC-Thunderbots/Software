#include "software/simulation/standalone_simulator.h"

#include "software/world/field.h"
extern "C"
{
#include "firmware/app/primitives/primitive.h"
}

StandaloneSimulator::StandaloneSimulator(
    std::shared_ptr<StandaloneSimulatorConfig> standalone_simulator_config)
    : standalone_simulator_config(standalone_simulator_config),
      simulator(Field::createSSLDivisionBField())
{
    standalone_simulator_config->mutableBlueTeamChannel()->registerCallbackFunction(
        [this](int) { this->initNetworking(); });
    standalone_simulator_config->mutableYellowTeamChannel()->registerCallbackFunction(
        [this](int) { this->initNetworking(); });
    standalone_simulator_config->mutableNetworkInterface()->registerCallbackFunction(
        [this](std::string) { this->initNetworking(); });
    standalone_simulator_config->mutableVisionPort()->registerCallbackFunction(
        [this](int) { this->initNetworking(); });
    standalone_simulator_config->mutableVisionIPv4Address()->registerCallbackFunction(
        [this](std::string) { this->initNetworking(); });

    initNetworking();

    simulator.registerOnSSLWrapperPacketReadyCallback(
        [this](SSL_WrapperPacket wrapper_packet) {
            this->wrapper_packet_sender->sendProto(wrapper_packet);
        });

    simulator.setBallState(BallState(Point(0, 0), Vector(0, 0)));

    simulator.startSimulation();
}

void StandaloneSimulator::registerOnSSLWrapperPacketReadyCallback(
    const std::function<void(SSL_WrapperPacket)>& callback)
{
    simulator.registerOnSSLWrapperPacketReadyCallback(callback);
}

void StandaloneSimulator::initNetworking()
{
    auto network_interface  = standalone_simulator_config->NetworkInterface()->value();
    int yellow_team_channel = standalone_simulator_config->YellowTeamChannel()->value();
    std::string yellow_team_ip =
        std::string(MULTICAST_CHANNELS[yellow_team_channel]) + "%" + network_interface;
    int blue_team_channel = standalone_simulator_config->BlueTeamChannel()->value();
    std::string blue_team_ip =
        std::string(MULTICAST_CHANNELS[blue_team_channel]) + "%" + network_interface;

    wrapper_packet_sender =
        std::make_unique<ThreadedProtoMulticastSender<SSL_WrapperPacket>>(
            standalone_simulator_config->VisionIPv4Address()->value(),
            static_cast<unsigned short>(
                standalone_simulator_config->VisionPort()->value()));
    yellow_team_primitive_listener =
        std::make_unique<ThreadedNanoPbPrimitiveSetMulticastListener>(
            yellow_team_ip, PRIMITIVE_PORT,
            boost::bind(&StandaloneSimulator::setYellowRobotPrimitives, this, _1));
    blue_team_primitive_listener =
        std::make_unique<ThreadedNanoPbPrimitiveSetMulticastListener>(
            blue_team_ip, PRIMITIVE_PORT,
            boost::bind(&StandaloneSimulator::setBlueRobotPrimitives, this, _1));
}

void StandaloneSimulator::setupInitialSimulationState()
{
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
}

void StandaloneSimulator::setYellowRobotPrimitives(
    std::map<RobotId, PrimitiveMsg> robot_id_to_primitive)
{
    for (const auto& robot_id_and_primitive : robot_id_to_primitive)
    {
        RobotId id             = robot_id_and_primitive.first;
        PrimitiveMsg primitive = robot_id_and_primitive.second;
        simulator.setYellowRobotPrimitive(id, primitive);
    }
}

void StandaloneSimulator::setBlueRobotPrimitives(
    std::map<RobotId, PrimitiveMsg> robot_id_to_primitive)
{
    for (const auto& robot_id_and_primitive : robot_id_to_primitive)
    {
        RobotId id             = robot_id_and_primitive.first;
        PrimitiveMsg primitive = robot_id_and_primitive.second;
        simulator.setBlueRobotPrimitive(id, primitive);
    }
}
