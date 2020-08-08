#include "software/backend/backend.h"

void Backend::receiveRobotStatusMsg(RobotStatusMsg msg)
{
    SensorMsg sensor_msg;
    *(sensor_msg.add_robot_status_msgs()) = msg;
    Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}

void Backend::receiveSSLWrapperPacket(SSL_WrapperPacket msg)
{
    SensorMsg sensor_msg;
    *(sensor_msg.mutable_ssl_vision_msg()) = msg;
    Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}

void Backend::receiveSSLReferee(SSL::SSL_Referee msg)
{
    SensorMsg sensor_msg;
    *(sensor_msg.mutable_ssl_referee_msg()) = msg;
    Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}
