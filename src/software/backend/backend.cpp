#include "software/backend/backend.h"

#include "software/proto/message_translation/tbots_protobuf.h"



Backend::Backend(): FirstInFirstOutThreadedObserver<World>(Observer<World>::DEFAULT_BUFFER_SIZE, false){}

void Backend::receiveRobotStatus(TbotsProto::RobotStatus msg)
{
    SensorProto sensor_msg;
    *(sensor_msg.add_robot_status_msgs())         = msg;
    *(sensor_msg.mutable_backend_received_time()) = *createCurrentTimestamp();
    Subject<SensorProto>::sendValueToObservers(sensor_msg);
}

void Backend::receiveSSLWrapperPacket(SSLProto::SSL_WrapperPacket msg)
{
    SensorProto sensor_msg;
    *(sensor_msg.mutable_ssl_vision_msg())        = msg;
    *(sensor_msg.mutable_backend_received_time()) = *createCurrentTimestamp();
    Subject<SensorProto>::sendValueToObservers(sensor_msg);
}

void Backend::receiveSSLReferee(SSLProto::Referee msg)
{
    SensorProto sensor_msg;
    *(sensor_msg.mutable_ssl_referee_msg())       = msg;
    *(sensor_msg.mutable_backend_received_time()) = *createCurrentTimestamp();
    Subject<SensorProto>::sendValueToObservers(sensor_msg);
}
