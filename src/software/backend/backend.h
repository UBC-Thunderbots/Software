#pragma once

#include "software/backend/robot_status.h"
#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
#include "software/primitive/primitive.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/world/world.h"

/**
 * A Backend is an abstraction around communication with robots, vision, and refbox. It
 * produces SensorMsgs, and consumes primitives that can be sent to the robots.
 *
 * This produce/consume pattern is performed by extending both "Observer" and
 * "Subject". Please see the implementation of those classes for details.
 */
class Backend : public Subject<SensorMsg>,
                public Subject<World>,
                public Subject<RobotStatus>,
                public ThreadedObserver<World>,
                public ThreadedObserver<ConstPrimitiveVectorPtr>
{
   public:
    Backend() = default;

    virtual ~Backend() = default;

    /**
     * Callback function to send components of SensorMsg via Subject<SensorMsg>
     * Immediately makes a SensorMsg from msg and sends it to Observers
     *
     * @param msg The component of SensorMsg
     */
    void receiveTbotsRobotMsg(TbotsRobotMsg msg);
    void receiveSSLWrapperPacket(SSL_WrapperPacket msg);
    void receiveSSLReferee(SSL_Referee msg);
};
