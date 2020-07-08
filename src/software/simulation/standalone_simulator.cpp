#include "software/simulation/standalone_simulator.h"

#include "software/world/field.h"
extern "C"
{
#include "firmware/app/primitives/primitive.h"
}

StandaloneSimulator::StandaloneSimulator(
    std::shared_ptr<StandaloneSimulatorConfig> standalone_simulator_config)
    : standalone_simulator_config(standalone_simulator_config),
      simulator(Field::createSSLDivisionBField(), 0.8, 0.2)
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

    simulator.setBallState(BallState(Point(0, 0), Vector(5, 2)));

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
        std::make_unique<ThreadedProtoMulticastListener<PrimitiveSetMsg>>(
            yellow_team_ip, PRIMITIVE_PORT,
            boost::bind(&StandaloneSimulator::setYellowRobotPrimitives, this, _1));
    blue_team_primitive_listener =
        std::make_unique<ThreadedProtoMulticastListener<PrimitiveSetMsg>>(
            blue_team_ip, PRIMITIVE_PORT,
            boost::bind(&StandaloneSimulator::setBlueRobotPrimitives, this, _1));
}

void StandaloneSimulator::setupInitialSimulationState()
{
    RobotState blue_robot_state1(Point(3, 2.5), Vector(0, 0), Angle::half(),
                                 AngularVelocity::zero());
    RobotState blue_robot_state2(Point(3, 1.5), Vector(0, 0), Angle::half(),
                                 AngularVelocity::zero());
    RobotState blue_robot_state3(Point(3, 0.5), Vector(0, 0), Angle::half(),
                                 AngularVelocity::zero());
    RobotState blue_robot_state4(Point(3, -0.5), Vector(0, 0), Angle::half(),
                                 AngularVelocity::zero());
    RobotState blue_robot_state5(Point(3, -1.5), Vector(0, 0), Angle::half(),
                                 AngularVelocity::zero());
    RobotState blue_robot_state6(Point(3, -2.5), Vector(0, 0), Angle::half(),
                                 AngularVelocity::zero());
    std::vector<RobotStateWithId> blue_robot_states = {
        RobotStateWithId{.id = 0, .robot_state = blue_robot_state1},
        RobotStateWithId{.id = 1, .robot_state = blue_robot_state2},
        RobotStateWithId{.id = 2, .robot_state = blue_robot_state3},
        RobotStateWithId{.id = 3, .robot_state = blue_robot_state4},
        RobotStateWithId{.id = 4, .robot_state = blue_robot_state5},
        RobotStateWithId{.id = 5, .robot_state = blue_robot_state6},
    };
    simulator.addBlueRobots(blue_robot_states);

    RobotState yellow_robot_state1(Point(-3, 2.5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    RobotState yellow_robot_state2(Point(-3, 1.5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    RobotState yellow_robot_state3(Point(-3, 0.5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    RobotState yellow_robot_state4(Point(-3, -0.5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    RobotState yellow_robot_state5(Point(-3, -1.5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    RobotState yellow_robot_state6(Point(-3, -2.5), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    std::vector<RobotStateWithId> yellow_robot_states = {
        RobotStateWithId{.id = 0, .robot_state = yellow_robot_state1},
        RobotStateWithId{.id = 1, .robot_state = yellow_robot_state2},
        RobotStateWithId{.id = 2, .robot_state = yellow_robot_state3},
        RobotStateWithId{.id = 3, .robot_state = yellow_robot_state4},
        RobotStateWithId{.id = 4, .robot_state = yellow_robot_state5},
        RobotStateWithId{.id = 5, .robot_state = yellow_robot_state6},
    };
    simulator.addYellowRobots(yellow_robot_states);
}

void StandaloneSimulator::setYellowRobotPrimitives(PrimitiveSetMsg msg)
{
    for (const auto& primitive : msg.robot_primitives())
    {
        RobotId id                               = primitive.first;
        auto [primitive_index, primitive_params] = decodePrimitiveMsg(primitive.second);
        simulator.setYellowRobotPrimitive(id, primitive_index, primitive_params);
    }
}

void StandaloneSimulator::setBlueRobotPrimitives(PrimitiveSetMsg msg)
{
    for (const auto& primitive : msg.robot_primitives())
    {
        RobotId id                               = primitive.first;
        auto [primitive_index, primitive_params] = decodePrimitiveMsg(primitive.second);
        simulator.setBlueRobotPrimitive(id, primitive_index, primitive_params);
    }
}

std::pair<unsigned int, primitive_params_t> StandaloneSimulator::decodePrimitiveMsg(
    const PrimitiveMsg& msg)
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

void StandaloneSimulator::startSimulation()
{
    simulator.startSimulation();
}

void StandaloneSimulator::stopSimulation()
{
    simulator.stopSimulation();
}

void StandaloneSimulator::setSlowMotionMultiplier(double multiplier)
{
    simulator.setSlowMotionMultiplier(multiplier);
}

void StandaloneSimulator::resetSlowMotionMultiplier()
{
    simulator.resetSlowMotionMultiplier();
}

void StandaloneSimulator::setBallState(const BallState& state)
{
    simulator.setBallState(state);
}
