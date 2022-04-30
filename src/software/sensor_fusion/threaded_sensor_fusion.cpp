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

//TODO This function should never see the light of day
//   if you see this in a PR, I've messed up, badly
//   you need to be protected from this horrible unsafe code
void ThreadedSensorFusion::updateGameState(const GameState &game_state)
{
    sensor_fusion.updateGameState(game_state);
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
