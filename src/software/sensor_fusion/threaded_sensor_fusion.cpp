#include "software/sensor_fusion/threaded_sensor_fusion.h"

ThreadedSensorFusion::ThreadedSensorFusion(
    std::shared_ptr<const SensorFusionConfig> sensor_fusion_config)
    : FirstInFirstOutThreadedObserver<SensorProto>(DIFFERENT_GRSIM_FRAMES_RECEIVED),
      sensor_fusion(sensor_fusion_config)
{
    if (!sensor_fusion_config)
    {
        throw std::invalid_argument(
            "ThreadedSensorFusion created with null SensorFusionConfig");
    }
}

void ThreadedSensorFusion::onValueReceived(SensorProto sensor_msg)
{
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
