#pragma once

#include "software/sensor_fusion/sensor_fusion.h"
#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/world/world.h"

class ThreadedSensorFusion : public Subject<World>, public ThreadedObserver<SensorMsg>
{
   public:
    ThreadedSensorFusion();

    virtual ~ThreadedSensorFusion() = default;

    // Delete the copy and assignment operators because this class really shouldn't need
    // them and we don't want to risk doing anything nasty with the internal
    // multithreading this class potentially uses
    ThreadedSensorFusion &operator=(const ThreadedSensorFusion &) = delete;
    ThreadedSensorFusion(const ThreadedSensorFusion &)            = delete;

   private:
    void onValueReceived(SensorMsg sensor_msg) override;

    SensorFusion sensor_fusion;
};
