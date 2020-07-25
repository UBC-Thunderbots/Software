#include "software/backend/backend.h"

#include "software/proto/message_translation/tbots_protobuf.h"

void Backend::receiveTbotsRobotMsg(TbotsRobotMsg msg)
{
    SensorMsg sensor_msg;
    *(sensor_msg.add_tbots_robot_msgs())  = msg;
    (*sensor_msg.mutable_time_received()) = *createCurrentTimestampMsg();
    Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}

void Backend::receiveSSLWrapperPacket(SSL_WrapperPacket msg)
{
    SensorMsg sensor_msg;
    *(sensor_msg.mutable_ssl_vision_msg()) = msg;
    (*sensor_msg.mutable_time_received())  = *createCurrentTimestampMsg();
    Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}

void Backend::receiveSSLReferee(SSL_Referee msg)
{
    SensorMsg sensor_msg;
    *(sensor_msg.mutable_ssl_refbox_msg()) = msg;
    (*sensor_msg.mutable_time_received())  = *createCurrentTimestampMsg();
    Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}
