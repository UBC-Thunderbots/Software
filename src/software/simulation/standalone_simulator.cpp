#include "software/simulation/standalone_simulator.h"

#include "software/world/field.h"
extern "C"
{
#include "firmware/app/primitives/primitive.h"
}

StandaloneSimulator::StandaloneSimulator(
    std::shared_ptr<StandaloneSimulatorConfig> standalone_simulator_config)
    : standalone_simulator_config(standalone_simulator_config),
      simulator(Field::createSSLDivisionBField(), 0.8, 0.2),
      most_recent_ssl_wrapper_packet(SSLProto::SSL_WrapperPacket())
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
        std::make_unique<ThreadedProtoMulticastSender<SSLProto::SSL_WrapperPacket>>(
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
    TbotsProto_PrimitiveSet primitive_set_msg)
{
    simulator.setYellowRobotPrimitiveSet(primitive_set_msg);
}

void StandaloneSimulator::setBlueRobotPrimitives(
    TbotsProto_PrimitiveSet primitive_set_msg)
{
    simulator.setBlueRobotPrimitiveSet(primitive_set_msg);
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
