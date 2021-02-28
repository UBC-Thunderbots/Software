#pragma once

#include "software/multithreading/first_in_first_out_threaded_observer.h"
#include "software/multithreading/subject.h"
#include "shared/parameter_v2/cpp_dynamic_parameters.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/sensor_fusion/sensor_fusion.h"
#include "software/world/world.h"

class ThreadedSensorFusion : public Subject<World>,
                             public FirstInFirstOutThreadedObserver<SensorProto>
{
   public:
    explicit ThreadedSensorFusion(
        std::shared_ptr<const SensorFusionConfig> sensor_fusion_config);
    virtual ~ThreadedSensorFusion() = default;

   private:
    void onValueReceived(SensorProto sensor_msg) override;

    SensorFusion sensor_fusion;
};
