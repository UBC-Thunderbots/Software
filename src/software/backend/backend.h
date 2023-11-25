#pragma once

#include "proto/sensor_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/multithreading/first_in_first_out_threaded_observer.h"
#include "software/multithreading/subject.hpp"
#include "software/world/world.h"

/**
 * A Backend is an abstraction around communication with robots, vision, and the
 * gamecontroller (Referee). It produces SensorProtos, and consumes primitives that can be
 * sent to the robots.
 *
 * This produce/consume pattern is performed by extending both "Observer" and
 * "Subject". Please see the implementation of those classes for details.
 */
class Backend : public Subject<SensorProto>,
                public FirstInFirstOutThreadedObserver<World>,
                public FirstInFirstOutThreadedObserver<TbotsProto::PrimitiveSet>
{
   public:
    Backend() = default;

    virtual ~Backend() = default;

    /**
     * Callback function to send components of SensorProto via Subject<SensorProto>
     * Immediately makes a SensorProto from msg and sends it to Observers
     *
     * @param msg The component of SensorProto
     */
    void receiveRobotStatus(TbotsProto::RobotStatus msg);
    void receiveSSLWrapperPacket(SSLProto::SSL_WrapperPacket msg);
    void receiveSSLReferee(SSLProto::Referee msg);
    void receiveSensorProto(SensorProto sensor_msg);
};
