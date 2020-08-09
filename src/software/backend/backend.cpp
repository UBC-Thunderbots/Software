#include "software/backend/backend.h"

void Backend::receiveRobotStatus(TbotsProto::RobotStatus msg)
{
    SensorProto sensor_msg;
    *(sensor_msg.add_robot_status_msgs()) = msg;
    Subject<SensorProto>::sendValueToObservers(sensor_msg);
}

void Backend::receiveSSLWrapperPacket(SSL_WrapperPacket msg)
{
    SensorProto sensor_msg;
    *(sensor_msg.mutable_ssl_vision_msg()) = msg;
    Subject<SensorProto>::sendValueToObservers(sensor_msg);
}

void Backend::receiveSSLReferee(SSL_Referee msg)
{
    SensorProto sensor_msg;
    *(sensor_msg.mutable_ssl_refbox_msg()) = msg;
    Subject<SensorProto>::sendValueToObservers(sensor_msg);
}
