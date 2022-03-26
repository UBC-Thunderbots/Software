#include "software/backend/wifi_backend.h"

#include "proto/message_translation/defending_side.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/robot_log_msg.pb.h"
#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/constants.h"
#include "software/estop/threaded_estop_reader.h"
#include "software/logger/logger.h"
#include "software/uart/boost_uart_communication.h"
#include "software/util/generic_factory/generic_factory.h"


WifiBackend::WifiBackend(std::shared_ptr<const BackendConfig> config)
    : network_config(config->getWifiBackendConfig()->getNetworkConfig()),
      sensor_fusion_config(config->getWifiBackendConfig()->getSensorFusionConfig()),
      arduino_config(config->getWifiBackendConfig()->getArduinoConfig()),
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

    // setup estop
    std::unique_ptr<BoostUartCommunication> uart_device =
        std::make_unique<BoostUartCommunication>(ARDUINO_BAUD_RATE,
                                                 arduino_config->getPort()->value());
    estop_reader = std::make_unique<ThreadedEstopReader>(std::move(uart_device));

    // connect to current channel
    joinMulticastChannel(channel, network_interface);
}

void WifiBackend::onValueReceived(TbotsProto::PrimitiveSet primitives)
{
    // check if estop has been set
    if (estop_reader != nullptr && !estop_reader->isEstopPlay())
    {
        auto robot_primitives_map = primitives.mutable_robot_primitives();

        // override to stop primitive
        for (auto& primitive : *robot_primitives_map)
        {
            primitive.second = *createEstopPrimitive();
        }
    }

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
}

void WifiBackend::onValueReceived(World world)
{
    vision_output->sendProto(*createVision(world));
    LOG(VISUALIZE) << *createWorld(world);
}

void WifiBackend::receiveRobotLogs(TbotsProto::RobotLog log)
{
    LOG(INFO) << "[ROBOT " << log.robot_id() << " " << LogLevel_Name(log.log_level())
              << "]"
              << "[" << log.file_name() << ":" << log.line_number()
              << "]: " << log.log_msg() << std::endl;
}

void WifiBackend::joinMulticastChannel(int channel, const std::string& interface)
{
    vision_output.reset(new ThreadedProtoUdpSender<TbotsProto::Vision>(
        std::string(ROBOT_MULTICAST_CHANNELS[channel]) + "%" + interface, VISION_PORT,
        true));

    primitive_output.reset(new ThreadedProtoUdpSender<TbotsProto::PrimitiveSet>(
        std::string(ROBOT_MULTICAST_CHANNELS[channel]) + "%" + interface, PRIMITIVE_PORT,
        true));

    robot_status_input.reset(new ThreadedProtoUdpListener<TbotsProto::RobotStatus>(
        std::string(ROBOT_MULTICAST_CHANNELS[channel]) + "%" + interface,
        ROBOT_STATUS_PORT, boost::bind(&Backend::receiveRobotStatus, this, _1), true));

    robot_log_input.reset(new ThreadedProtoUdpListener<TbotsProto::RobotLog>(
        std::string(ROBOT_MULTICAST_CHANNELS[channel]) + "%" + interface, ROBOT_LOGS_PORT,
        boost::bind(&WifiBackend::receiveRobotLogs, this, _1), true));

    defending_side_output.reset(new ThreadedProtoUdpSender<DefendingSideProto>(
        std::string(ROBOT_MULTICAST_CHANNELS[channel]) + "%" + interface,
        DEFENDING_SIDE_PORT, true));
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, WifiBackend, BackendConfig> factory;
