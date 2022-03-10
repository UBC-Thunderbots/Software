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

UnixSimulatorBackend::UnixSimulatorBackend(std::shared_ptr<const BackendConfig> config)
    : sensor_fusion_config(config->getSimulatorBackendConfig()->getSensorFusionConfig())
{
    // Protobuf Inputs
    robot_status_input.reset(new ThreadedProtoUnixListener<TbotsProto::RobotStatus>(
        config->getFullSystemMainCommandLineArgs()->getRuntimeDir()->value() +
            ROBOT_STATUS_INPUT_PATH,
        boost::bind(&Backend::receiveRobotStatus, this, _1)));

    ssl_wrapper_input.reset(new ThreadedProtoUnixListener<SSLProto::SSL_WrapperPacket>(
        config->getFullSystemMainCommandLineArgs()->getRuntimeDir()->value() +
            SSL_WRAPPER_INPUT_PATH,
        boost::bind(&Backend::receiveSSLWrapperPacket, this, _1)));

    ssl_referee_input.reset(new ThreadedProtoUnixListener<SSLProto::Referee>(
        config->getFullSystemMainCommandLineArgs()->getRuntimeDir()->value() +
            SSL_REFEREE_INPUT_PATH,
        boost::bind(&Backend::receiveSSLReferee, this, _1)));

    sensor_proto_input.reset(new ThreadedProtoUnixListener<SensorProto>(
        config->getFullSystemMainCommandLineArgs()->getRuntimeDir()->value() +
            SENSOR_PROTO_INPUT_PATH,
        boost::bind(&Backend::receiveSensorProto, this, _1)));

    // Protobuf Outputs
    vision_output.reset(new ThreadedProtoUnixSender<TbotsProto::Vision>(
        config->getFullSystemMainCommandLineArgs()->getRuntimeDir()->value() +
        VISION_OUTPUT_PATH));

    primitive_output.reset(new ThreadedProtoUnixSender<TbotsProto::PrimitiveSet>(
        config->getFullSystemMainCommandLineArgs()->getRuntimeDir()->value() +
        PRIMITIVE_OUTPUT_PATH));

    defending_side_output.reset(new ThreadedProtoUnixSender<DefendingSideProto>(
        config->getFullSystemMainCommandLineArgs()->getRuntimeDir()->value() +
        DEFENDING_SIDE_OUTPUT));
}

void UnixSimulatorBackend::onValueReceived(TbotsProto::PrimitiveSet primitives)
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
        static_cast<float>(
            FirstInFirstOutThreadedObserver<World>::getDataReceivedPerSecond()));
}

void UnixSimulatorBackend::onValueReceived(World world)
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

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, UnixSimulatorBackend, BackendConfig> factory;
