#include "software/backend/backend.h"

#include "software/proto/message_translation/tbots_protobuf.h"

void Backend::receiveRobotStatus(TbotsProto::RobotStatus msg)
{
    SensorProto sensor_msg;
    *(sensor_msg.add_robot_status_msgs()) = msg;
    (*sensor_msg.mutable_time_received()) = *createCurrentTimestamp();
    Subject<SensorProto>::sendValueToObservers(sensor_msg);
}

void Backend::receiveSSLWrapperPacket(SSLProto::SSL_WrapperPacket msg)
{
    SensorProto sensor_msg;
    *(sensor_msg.mutable_ssl_vision_msg()) = msg;
    (*sensor_msg.mutable_time_received())  = *createCurrentTimestamp();
    Subject<SensorProto>::sendValueToObservers(sensor_msg);
}

void Backend::receiveSSLReferee(SSLProto::Referee msg)
{
    SensorProto sensor_msg;
    *(sensor_msg.mutable_ssl_referee_msg()) = msg;
    (*sensor_msg.mutable_time_received())   = *createCurrentTimestamp();
    Subject<SensorProto>::sendValueToObservers(sensor_msg);
}
