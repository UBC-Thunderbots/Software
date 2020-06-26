#include "software/backend/backend.h"

void Backend::receiveTbotsRobotMsg(TbotsRobotMsg msg)
{
    SensorMsg sensor_msg;
    *(sensor_msg.add_tbots_robot_msgs()) = msg;
    Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}

void Backend::receiveSSLWrapperPacket(SSL_WrapperPacket msg)
{
    SensorMsg sensor_msg;
    *(sensor_msg.mutable_ssl_vision_msg()) = msg;
    Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}

void Backend::receiveSSLReferee(SSL_Referee msg)
{
    SensorMsg sensor_msg;
    *(sensor_msg.mutable_ssl_refbox_msg()) = msg;
    Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}
