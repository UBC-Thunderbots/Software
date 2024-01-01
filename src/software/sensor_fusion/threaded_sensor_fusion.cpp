#include "software/sensor_fusion/threaded_sensor_fusion.h"

#include <google/protobuf/util/message_differencer.h>
#include <tracy/Tracy.hpp>

ThreadedSensorFusion::ThreadedSensorFusion(
    TbotsProto::SensorFusionConfig sensor_fusion_config)
    : FirstInFirstOutThreadedObserver<SensorProto>(DIFFERENT_GRSIM_FRAMES_RECEIVED),
      sensor_fusion(sensor_fusion_config)
{
}

void ThreadedSensorFusion::onValueReceived(TbotsProto::ThunderbotsConfig config)
{
    std::scoped_lock lock(sensor_fusion_mutex);

    // If we received a new SensorFusion, restart sensor fusion
    // with the new config
    if (!google::protobuf::util::MessageDifferencer::Equivalent(
            config.sensor_fusion_config(), sensor_fusion_config))
    {
        sensor_fusion = SensorFusion(config.sensor_fusion_config());
    }
}

void ThreadedSensorFusion::onValueReceived(SensorProto sensor_msg)
{
    std::scoped_lock lock(sensor_fusion_mutex);
    ZoneScopedN("ThreadedSensorFusion");
    sensor_fusion.processSensorProto(sensor_msg);

    // Limit sensor fusion to only send out worlds on ssl wrapper packets
    // to prevent spamming worlds every time a referee msg or robot status
    // msg comes through.
    if (sensor_msg.has_ssl_vision_msg())
    {
        std::optional<World> world = sensor_fusion.getWorld();
        if (world)
        {
            Subject<World>::sendValueToObservers(world.value());
        }
    }
}
