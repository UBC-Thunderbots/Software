#include "software/backend/unix_simulator_backend.h"

#include "proto/message_translation/defending_side.h"
#include "proto/message_translation/ssl_wrapper.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/robot_log_msg.pb.h"
#include "proto/sensor_msg.pb.h"
#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

SimulatorBackend::SimulatorBackend(std::shared_ptr<const BackendConfig> config)
    : sensor_fusion_config(config->getSimulatorBackendConfig()->getSensorFusionConfig())
{
    // Protobuf Inputs
    robot_status_input.reset(new ThreadedProtoUnixListener<TbotsProto::RobotStatus>(
        config->getSimulatorBackendConfig()->getBaseUnixPath()->value() +
            ROBOT_STATUS_INPUT_PATH,
        boost::bind(&Backend::receiveRobotStatus, this, _1)));

    ssl_wrapper_input.reset(new ThreadedProtoUnixListener<SSLProto::SSL_WrapperPacket>(
        config->getSimulatorBackendConfig()->getBaseUnixPath()->value() +
            SSL_WRAPPER_INPUT_PATH,
        boost::bind(&Backend::receiveSSLWrapperPacket, this, _1)));

    ssl_referee_input.reset(new ThreadedProtoUnixListener<SSLProto::Referee>(
        config->getSimulatorBackendConfig()->getBaseUnixPath()->value() +
            SSL_REFEREE_INPUT_PATH,
        boost::bind(&Backend::receiveSSLReferee, this, _1)));

    sensor_proto_input.reset(new ThreadedProtoUnixListener<SensorProto>(
        config->getSimulatorBackendConfig()->getBaseUnixPath()->value() +
            SENSOR_PROTO_INPUT_PATH,
        boost::bind(&Backend::receiveSensorProto, this, _1)));

    // Protobuf Outputs
    vision_output.reset(new ThreadedProtoUnixSender<TbotsProto::Vision>(
        config->getSimulatorBackendConfig()->getBaseUnixPath()->value() +
        VISION_OUTPUT_PATH));

    primitive_output.reset(new ThreadedProtoUnixSender<TbotsProto::PrimitiveSet>(
        config->getSimulatorBackendConfig()->getBaseUnixPath()->value() +
        PRIMITIVE_OUTPUT_PATH));

    defending_side_output.reset(new ThreadedProtoUnixSender<DefendingSideProto>(
        config->getSimulatorBackendConfig()->getBaseUnixPath()->value() +
        DEFENDING_SIDE_OUTPUT));
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
}

void SimulatorBackend::onValueReceived(World world)
{
    vision_output->sendProto(*createVision(world));
    LOG(VISUALIZE) << *createWorld(world);
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, SimulatorBackend, BackendConfig> factory;
