#include "software/simulation/standalone_simulator.h"

#include "software/proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "software/world/field.h"
extern "C"
{
#include "firmware/app/primitives/primitive.h"
}

StandaloneSimulator::StandaloneSimulator(
    std::shared_ptr<StandaloneSimulatorConfig> standalone_simulator_config,
    std::shared_ptr<SimulatorConfig> simulator_config)
    : standalone_simulator_config(standalone_simulator_config),
      simulator(Field::createSSLDivisionBField(), simulator_config),
      most_recent_ssl_wrapper_packet(SSLProto::SSL_WrapperPacket())
{
    standalone_simulator_config->getMutableBlueTeamChannel()->registerCallbackFunction(
        [this](int blue_team_channel) {
            setupNetworking(
                blue_team_channel,
                this->standalone_simulator_config->getYellowTeamChannel()->value(),
                this->standalone_simulator_config->getNetworkInterface()->value(),
                this->standalone_simulator_config->getVisionPort()->value(),
                this->standalone_simulator_config->getVisionIpv4Address()->value());
        });
    standalone_simulator_config->getMutableYellowTeamChannel()->registerCallbackFunction(
        [this](int yellow_team_channel) {
            setupNetworking(
                this->standalone_simulator_config->getBlueTeamChannel()->value(),
                yellow_team_channel,
                this->standalone_simulator_config->getNetworkInterface()->value(),
                this->standalone_simulator_config->getVisionPort()->value(),
                this->standalone_simulator_config->getVisionIpv4Address()->value());
        });
    standalone_simulator_config->getMutableNetworkInterface()->registerCallbackFunction(
        [this](std::string network_interface) {
            setupNetworking(
                this->standalone_simulator_config->getBlueTeamChannel()->value(),
                this->standalone_simulator_config->getYellowTeamChannel()->value(),
                network_interface,
                this->standalone_simulator_config->getVisionPort()->value(),
                this->standalone_simulator_config->getVisionIpv4Address()->value());
        });
    standalone_simulator_config->getMutableVisionPort()->registerCallbackFunction(
        [this](int vision_port) {
            setupNetworking(
                this->standalone_simulator_config->getBlueTeamChannel()->value(),
                this->standalone_simulator_config->getYellowTeamChannel()->value(),
                this->standalone_simulator_config->getNetworkInterface()->value(),
                vision_port,
                this->standalone_simulator_config->getVisionIpv4Address()->value());
        });
    standalone_simulator_config->getMutableVisionIpv4Address()->registerCallbackFunction(
        [this](std::string vision_ip_address) {
            setupNetworking(
                this->standalone_simulator_config->getBlueTeamChannel()->value(),
                this->standalone_simulator_config->getYellowTeamChannel()->value(),
                this->standalone_simulator_config->getNetworkInterface()->value(),
                this->standalone_simulator_config->getVisionPort()->value(),
                vision_ip_address);
        });

    setupNetworking(standalone_simulator_config->getBlueTeamChannel()->value(),
                    standalone_simulator_config->getYellowTeamChannel()->value(),
                    standalone_simulator_config->getNetworkInterface()->value(),
                    standalone_simulator_config->getVisionPort()->value(),
                    standalone_simulator_config->getVisionIpv4Address()->value());

    simulator.registerOnSSLWrapperPacketReadyCallback(
        [this](SSLProto::SSL_WrapperPacket wrapper_packet) {
            std::scoped_lock lock(this->most_recent_ssl_wrapper_packet_mutex);
            this->most_recent_ssl_wrapper_packet = wrapper_packet;
            this->wrapper_packet_sender->sendProto(wrapper_packet);
        });

    simulator.setBallState(BallState(Point(0, 0), Vector(5, 2)));

    simulator.startSimulation();
}

void StandaloneSimulator::registerOnSSLWrapperPacketReadyCallback(
    const std::function<void(SSLProto::SSL_WrapperPacket)>& callback)
{
    simulator.registerOnSSLWrapperPacketReadyCallback(callback);
}

void StandaloneSimulator::setupNetworking(int blue_team_channel, int yellow_team_channel,
                                          std::string network_interface, int vision_port,
                                          std::string vision_ip_address)
{
    std::string yellow_team_ip =
        std::string(MULTICAST_CHANNELS[yellow_team_channel]) + "%" + network_interface;
    std::string blue_team_ip =
        std::string(MULTICAST_CHANNELS[blue_team_channel]) + "%" + network_interface;

    wrapper_packet_sender.reset(new ThreadedProtoUdpSender<SSLProto::SSL_WrapperPacket>(
        vision_ip_address, static_cast<unsigned short>(vision_port), true));
    yellow_team_primitive_listener.reset(
        new ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>(
            yellow_team_ip, PRIMITIVE_PORT,
            boost::bind(&StandaloneSimulator::setYellowRobotPrimitives, this, _1)));
    blue_team_primitive_listener.reset(
        new ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>(
            blue_team_ip, PRIMITIVE_PORT,
            boost::bind(&StandaloneSimulator::setBlueRobotPrimitives, this, _1)));
    yellow_team_side_listener.reset(new ThreadedProtoUdpListener<DefendingSideProto>(
        yellow_team_ip, DEFENDING_SIDE_PORT,
        boost::bind(&StandaloneSimulator::setYellowTeamDefendingSide, this, _1)));
    blue_team_side_listener.reset(new ThreadedProtoUdpListener<DefendingSideProto>(
        blue_team_ip, DEFENDING_SIDE_PORT,
        boost::bind(&StandaloneSimulator::setBlueTeamDefendingSide, this, _1)));
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

SSLProto::SSL_WrapperPacket StandaloneSimulator::getSSLWrapperPacket() const
{
    std::scoped_lock lock(most_recent_ssl_wrapper_packet_mutex);
    return most_recent_ssl_wrapper_packet;
}

void StandaloneSimulator::setYellowRobotPrimitives(
    const TbotsProto::PrimitiveSet& primitive_set_msg)
{
    simulator.setYellowRobotPrimitiveSet(createNanoPbPrimitiveSet(primitive_set_msg));
}

void StandaloneSimulator::setBlueRobotPrimitives(
    const TbotsProto::PrimitiveSet& primitive_set_msg)
{
    simulator.setBlueRobotPrimitiveSet(createNanoPbPrimitiveSet(primitive_set_msg));
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

std::weak_ptr<PhysicsRobot> StandaloneSimulator::getRobotAtPosition(const Point& position)
{
    return simulator.getRobotAtPosition(position);
}

void StandaloneSimulator::addYellowRobot(const Point& position)
{
    simulator.addYellowRobot(position);
}

void StandaloneSimulator::addBlueRobot(const Point& position)
{
    simulator.addBlueRobot(position);
}

void StandaloneSimulator::removeRobot(std::weak_ptr<PhysicsRobot> robot)
{
    simulator.removeRobot(robot);
}

void StandaloneSimulator::setBlueTeamDefendingSide(
    const DefendingSideProto& defending_side_proto)
{
    simulator.setBlueTeamDefendingSide(defending_side_proto);
}

void StandaloneSimulator::setYellowTeamDefendingSide(
    const DefendingSideProto& defending_side_proto)
{
    simulator.setYellowTeamDefendingSide(defending_side_proto);
}
