#include "software/backend/simulator_backend.h"

#include "proto/message_translation/defending_side.h"
#include "proto/message_translation/ssl_wrapper.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/robot_log_msg.pb.h"
#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

SimulatorBackend::SimulatorBackend(std::shared_ptr<const BackendConfig> config)
    : network_config(config->getSimulatorBackendConfig()->getNetworkConfig()),
      sensor_fusion_config(config->getSimulatorBackendConfig()->getSensorFusionConfig()),
      ssl_proto_client(boost::bind(&Backend::receiveSSLWrapperPacket, this, _1),
                       boost::bind(&Backend::receiveSSLReferee, this, _1),
                       network_config->getSslCommunicationConfig())
{
    std::string network_interface = this->network_config->getNetworkInterface()->value();
    int channel                   = this->network_config->getChannel()->value();

    network_config->getChannel()->registerCallbackFunction([this](int new_channel) {
        std::string new_network_interface =
            this->network_config->getNetworkInterface()->value();
        joinMulticastChannel(new_channel, new_network_interface);
    });

    // connect to current channel
    joinMulticastChannel(channel, network_interface);
}

void SimulatorBackend::onValueReceived(TbotsProto::PrimitiveSet primitives)
{
    primitive_output->sendProto(primitives);

    if (sensor_fusion_config->getOverrideGameControllerDefendingSide()->value())
    {
        defending_side_output->sendProto(
            *createDefendingSide(sensor_fusion_config->getDefendingPositiveSide()->value()
                                     ? FieldSide::POS_X
                                     : FieldSide::NEG_X));
    }
    else
    {
        defending_side_output->sendProto(*createDefendingSide(FieldSide::NEG_X));
    }
    // TODO (#2510) Find a new home once SimulatorBackend and ThreadedFullSystemGUI are
    // gone
    LOG(VISUALIZE) << *createNamedValue(
        "Primitive Hz",
        static_cast<float>(FirstInFirstOutThreadedObserver<
                           TbotsProto::PrimitiveSet>::getDataReceivedPerSecond()));
}

void SimulatorBackend::onValueReceived(World world)
{
    vision_output->sendProto(*createVision(world));
    LOG(VISUALIZE) << *createWorld(world);
    // TODO (#2510) Find a new home once SimulatorBackend and ThreadedFullSystemGUI are
    // gone
    LOG(VISUALIZE) << *createNamedValue(
        "World Hz",
        static_cast<float>(
            FirstInFirstOutThreadedObserver<World>::getDataReceivedPerSecond()));
}

void SimulatorBackend::receiveRobotLogs(TbotsProto::RobotLog log)
{
    LOG(INFO) << "[ROBOT " << log.robot_id() << " " << LogLevel_Name(log.log_level())
              << "]"
              << "[" << log.file_name() << ":" << log.line_number()
              << "]: " << log.log_msg() << std::endl;
}

void SimulatorBackend::joinMulticastChannel(int channel, const std::string& interface)
{
    vision_output.reset(new ThreadedProtoUdpSender<TbotsProto::Vision>(
        std::string(SIMULATOR_MULTICAST_CHANNELS[channel]) + "%" + interface, VISION_PORT,
        true));

    primitive_output.reset(new ThreadedProtoUdpSender<TbotsProto::PrimitiveSet>(
        std::string(SIMULATOR_MULTICAST_CHANNELS[channel]) + "%" + interface,
        PRIMITIVE_PORT, true));

    robot_status_input.reset(new ThreadedProtoUdpListener<TbotsProto::RobotStatus>(
        std::string(SIMULATOR_MULTICAST_CHANNELS[channel]) + "%" + interface,
        ROBOT_STATUS_PORT, boost::bind(&Backend::receiveRobotStatus, this, _1), true));

    robot_log_input.reset(new ThreadedProtoUdpListener<TbotsProto::RobotLog>(
        std::string(SIMULATOR_MULTICAST_CHANNELS[channel]) + "%" + interface,
        ROBOT_LOGS_PORT, boost::bind(&SimulatorBackend::receiveRobotLogs, this, _1),
        true));

    defending_side_output.reset(new ThreadedProtoUdpSender<DefendingSideProto>(
        std::string(SIMULATOR_MULTICAST_CHANNELS[channel]) + "%" + interface,
        DEFENDING_SIDE_PORT, true));
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, SimulatorBackend, BackendConfig> factory;
