#include "software/backend/simulator_backend.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "shared/proto/robot_log_msg.pb.h"
#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/proto/message_translation/defending_side.h"
#include "software/proto/message_translation/tbots_protobuf.h"
#include "software/util/design_patterns/generic_factory.h"

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

    // For some reason: defending_side is segfaulting
    // TODO (#2167) fix networking stuff and figure out why proto is broken
    // if (sensor_fusion_config->getOverrideGameControllerDefendingSide()->value())
    // {
    //     defending_side_output->sendProto(
    //         *createDefendingSide(sensor_fusion_config->getDefendingPositiveSide()->value()
    //                                  ? FieldSide::POS_X
    //                                  : FieldSide::NEG_X));
    // }
    // else
    // {
    //     defending_side_output->sendProto(*createDefendingSide(FieldSide::NEG_X));
    // }
}

void SimulatorBackend::onValueReceived(World world)
{
    vision_output->sendProto(*createVision(world));
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
        std::string(SIMULATOR_MULTICAST_CHANNELS[channel]), VISION_PORT, true));

    primitive_output.reset(new ThreadedProtoUdpSender<TbotsProto::PrimitiveSet>(
        std::string(SIMULATOR_MULTICAST_CHANNELS[channel]), PRIMITIVE_PORT, true));

    robot_status_input.reset(new ThreadedProtoUdpListener<TbotsProto::RobotStatus>(
        std::string(SIMULATOR_MULTICAST_CHANNELS[channel]), ROBOT_STATUS_PORT,
        boost::bind(&Backend::receiveRobotStatus, this, _1), true));

    robot_log_input.reset(new ThreadedProtoUdpListener<TbotsProto::RobotLog>(
        std::string(SIMULATOR_MULTICAST_CHANNELS[channel]), ROBOT_LOGS_PORT,
        boost::bind(&SimulatorBackend::receiveRobotLogs, this, _1), true));

    // For some reason
    // defending_side_output.reset(new ThreadedProtoUdpSender<DefendingSideProto>(
    //     std::string(SIMULATOR_MULTICAST_CHANNELS[channel]), DEFENDING_SIDE_PORT, true));
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, SimulatorBackend, BackendConfig> factory;
