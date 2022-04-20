#include "software/backend/unix_simulator_backend.h"

#include "proto/message_translation/defending_side.h"
#include "proto/message_translation/ssl_wrapper.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/parameters.pb.h"
#include "proto/robot_log_msg.pb.h"
#include "proto/sensor_msg.pb.h"
#include "shared/constants.h"
#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

UnixSimulatorBackend::UnixSimulatorBackend(std::string runtime_dir)
{
    const TbotsProto::SensorFusionConfig sensor_fusion_config;

    // Protobuf Inputs
    robot_status_input.reset(new ThreadedProtoUnixListener<TbotsProto::RobotStatus>(
        runtime_dir + ROBOT_STATUS_PATH,
        boost::bind(&Backend::receiveRobotStatus, this, _1)));

    ssl_wrapper_input.reset(new ThreadedProtoUnixListener<SSLProto::SSL_WrapperPacket>(
        runtime_dir + SSL_WRAPPER_PATH,
        boost::bind(&Backend::receiveSSLWrapperPacket, this, _1)));

    ssl_referee_input.reset(new ThreadedProtoUnixListener<SSLProto::Referee>(
        runtime_dir + SSL_REFEREE_PATH,
        boost::bind(&Backend::receiveSSLReferee, this, _1)));

    sensor_proto_input.reset(new ThreadedProtoUnixListener<SensorProto>(
        runtime_dir + SENSOR_PROTO_PATH,
        boost::bind(&Backend::receiveSensorProto, this, _1)));

    // Protobuf Outputs
    world_output.reset(
        new ThreadedProtoUnixSender<TbotsProto::World>(runtime_dir + WORLD_PATH));

    primitive_output.reset(new ThreadedProtoUnixSender<TbotsProto::PrimitiveSet>(
        runtime_dir + PRIMITIVE_PATH));
}

void UnixSimulatorBackend::onValueReceived(TbotsProto::PrimitiveSet primitives)
{
    primitive_output->sendProto(primitives);

    // TODO (#2510) Find a new home once SimulatorBackend and ThreadedFullSystemGUI are
    // gone
    LOG(VISUALIZE) << *createNamedValue(
        "Primitive Hz",
        static_cast<float>(FirstInFirstOutThreadedObserver<
                           TbotsProto::PrimitiveSet>::getDataReceivedPerSecond()));
}

void UnixSimulatorBackend::onValueReceived(World world)
{
    world_output->sendProto(*createWorld(world));
    // TODO (#2510) Find a new home once SimulatorBackend and ThreadedFullSystemGUI are
    // gone
    LOG(VISUALIZE) << *createNamedValue(
        "World Hz",
        static_cast<float>(
            FirstInFirstOutThreadedObserver<World>::getDataReceivedPerSecond()));
}
