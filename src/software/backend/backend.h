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
                public ThreadedObserver<ConstPrimitiveVectorPtr>
{
   public:
    Backend() = default;

    virtual ~Backend() = default;

    // Delete the copy and assignment operators because this class really shouldn't need
    // them and we don't want to risk doing anything nasty with the internal
    // multithreading this class potentially uses
    Backend& operator=(const Backend&) = delete;
    Backend(const Backend&)            = delete;
};
