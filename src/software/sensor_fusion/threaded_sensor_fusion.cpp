#include "software/sensor_fusion/threaded_sensor_fusion.h"

ThreadedSensorFusion::ThreadedSensorFusion()
    : sensor_fusion()
{
}

void ThreadedSensorFusion::onValueReceived(SensorMsg sensor_msg)
{
    sensor_fusion.updateWorld(sensor_msg);
    std::optional<World> world = sensor_fusion.getWorld();
    if(world) {
        Subject<World>::sendValueToObservers(world.value());
    }
}
