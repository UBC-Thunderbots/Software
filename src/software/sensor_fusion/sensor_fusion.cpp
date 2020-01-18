#include "software/sensor_fusion/sensor_fusion.h"

SensorFusion::SensorFusion()
//    : network_input(Util::Constants::SSL_VISION_DEFAULT_MULTICAST_ADDRESS,
//                    Util::Constants::SSL_VISION_MULTICAST_PORT,
//                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
//                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_PORT,
//                    boost::bind(&RadioBackend::receiveWorld, this, _1)),
//      radio_output(DEFAULT_RADIO_CONFIG, [this](RobotStatus status) {
//          Subject<RobotStatus>::sendValueToObservers(status);
//      })
{
}

void SensorFusion::onValueReceived(RefboxData refbox_data)
{
    updateWorld(refbox_data);
    Subject<World>::sendValueToObservers(world);
}

void SensorFusion::onValueReceived(RobotStatus robot_status)
{
    updateWorld(robot_status);
    Subject<World>::sendValueToObservers(world);
}

void SensorFusion::onValueReceived(VisionDetection vision_detection)
{
    updateWorld(vision_detection);
    Subject<World>::sendValueToObservers(world);
}

void updateWorld(RefboxData refbox_data) {}

void updateWorld(RobotStatus robot_status) {}

void updateWorld(VisionDetection vision_detection) {}
