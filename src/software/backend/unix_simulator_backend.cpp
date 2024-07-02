#include "software/backend/unix_simulator_backend.h"

#include "proto/message_translation/ssl_wrapper.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/parameters.pb.h"
#include "proto/robot_log_msg.pb.h"
#include "proto/sensor_msg.pb.h"
#include "shared/constants.h"
#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

UnixSimulatorBackend::UnixSimulatorBackend(
    std::string runtime_dir, const std::shared_ptr<ProtoLogger>& proto_logger)
    : proto_logger(proto_logger)
{
    // Protobuf Inputs
    robot_status_input.reset(new ThreadedProtoUnixListener<TbotsProto::RobotStatus>(
        runtime_dir + ROBOT_STATUS_PATH,
        boost::bind(&Backend::receiveRobotStatus, this, _1), proto_logger));

    ssl_wrapper_input.reset(new ThreadedProtoUnixListener<SSLProto::SSL_WrapperPacket>(
        runtime_dir + SSL_WRAPPER_PATH,
        boost::bind(&Backend::receiveSSLWrapperPacket, this, _1), proto_logger));

    ssl_referee_input.reset(new ThreadedProtoUnixListener<SSLProto::Referee>(
        runtime_dir + SSL_REFEREE_PATH,
        boost::bind(&Backend::receiveSSLReferee, this, _1), proto_logger));

    sensor_proto_input.reset(new ThreadedProtoUnixListener<SensorProto>(
        runtime_dir + SENSOR_PROTO_PATH,
        boost::bind(&Backend::receiveSensorProto, this, _1), proto_logger));

    dynamic_parameter_update_request_listener.reset(
        new ThreadedProtoUnixListener<TbotsProto::ThunderbotsConfig>(
            runtime_dir + DYNAMIC_PARAMETER_UPDATE_REQUEST_PATH,
            boost::bind(&UnixSimulatorBackend::receiveThunderbotsConfig, this, _1),
            proto_logger));

    // Empty callback for ValidationProtoSet since it's only used by proto_logger
    validation_proto_set_listener.reset(
        new ThreadedProtoUnixListener<TbotsProto::ValidationProtoSet>(
            runtime_dir + VALIDATION_PROTO_SET_PATH,
            [](TbotsProto::ValidationProtoSet v) {}, proto_logger));

    // Protobuf Outputs
    world_output.reset(new ThreadedProtoUnixSender<TbotsProto::World>(
        runtime_dir + WORLD_PATH, proto_logger));

    primitive_output.reset(new ThreadedProtoUnixSender<TbotsProto::PrimitiveSet>(
        runtime_dir + PRIMITIVE_PATH, proto_logger));

    dynamic_parameter_update_respone_sender.reset(
        new ThreadedProtoUnixSender<TbotsProto::ThunderbotsConfig>(
            runtime_dir + DYNAMIC_PARAMETER_UPDATE_RESPONSE_PATH, proto_logger));
}

void UnixSimulatorBackend::receiveThunderbotsConfig(TbotsProto::ThunderbotsConfig request)
{
    // Send new config to all observers
    Subject<TbotsProto::ThunderbotsConfig>::sendValueToObservers(request);

    // Echo back the request as an acknowledge
    dynamic_parameter_update_respone_sender->sendProto(request);
}

void UnixSimulatorBackend::onValueReceived(TbotsProto::PrimitiveSet primitives)
{
    primitive_output->sendProto(primitives);

    LOG(VISUALIZE) << *createNamedValue(
        "Primitive Hz",
        static_cast<float>(FirstInFirstOutThreadedObserver<
                           TbotsProto::PrimitiveSet>::getDataReceivedPerSecond()));
}

void UnixSimulatorBackend::onValueReceived(World world)
{
    world_output->sendProto(*createWorldWithSequenceNumber(world, sequence_number++));

    LOG(VISUALIZE) << *createNamedValue(
        "World Hz",
        static_cast<float>(
            FirstInFirstOutThreadedObserver<World>::getDataReceivedPerSecond()));

    std::scoped_lock lock(last_world_time_mutex);
    last_world_time_sec = world.getMostRecentTimestamp().toSeconds();
}

double UnixSimulatorBackend::getLastWorldTimeSec()
{
    std::scoped_lock lock(last_world_time_mutex);
    return last_world_time_sec;
}
