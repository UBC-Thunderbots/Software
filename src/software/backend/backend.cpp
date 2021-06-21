#include "software/backend/backend.h"

#include "software/proto/message_translation/tbots_protobuf.h"



Backend::Backend()
    // TODO (#2167): is there better way to remove warnings
    : FirstInFirstOutThreadedObserver<World>(Observer<World>::DEFAULT_BUFFER_SIZE, false),
      accumulated_robot_statuses()
{
}

void Backend::receiveRobotStatus(TbotsProto::RobotStatus msg)
{
    SensorProto sensor_msg;
    *(accumulated_robot_statuses.Add()) = msg;
}

void Backend::receiveSSLWrapperPacket(SSLProto::SSL_WrapperPacket msg)
{
    SensorProto sensor_msg;
    // TODO (#2167): Only send robot status only upon receiving vision
    *(sensor_msg.mutable_robot_status_msgs())     = accumulated_robot_statuses;
    *(sensor_msg.mutable_ssl_vision_msg())        = msg;
    *(sensor_msg.mutable_backend_received_time()) = *createCurrentTimestamp();
    Subject<SensorProto>::sendValueToObservers(sensor_msg);

    // Reset accumulated robot statuses
    accumulated_robot_statuses =
        google::protobuf::RepeatedPtrField<TbotsProto::RobotStatus>();
}

void Backend::receiveSSLReferee(SSLProto::Referee msg)
{
    SensorProto sensor_msg;
    *(sensor_msg.mutable_ssl_referee_msg())       = msg;
    *(sensor_msg.mutable_backend_received_time()) = *createCurrentTimestamp();
    Subject<SensorProto>::sendValueToObservers(sensor_msg);
}
