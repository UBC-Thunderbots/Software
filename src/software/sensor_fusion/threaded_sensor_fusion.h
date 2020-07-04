#pragma once

#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/sensor_fusion/sensor_fusion.h"
#include "software/world/world.h"

class ThreadedSensorFusion : public Subject<World>, public ThreadedObserver<SensorMsg>
{
   public:
    explicit ThreadedSensorFusion(
        std::shared_ptr<const SensorFusionConfig> sensor_fusion_config =
            DynamicParameters->getSensorFusionConfig());
    virtual ~ThreadedSensorFusion() = default;

   private:
    void onValueReceived(SensorMsg sensor_msg) override;

    SensorFusion sensor_fusion;
};
