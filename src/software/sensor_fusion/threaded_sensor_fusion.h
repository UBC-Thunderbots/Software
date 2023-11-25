#pragma once

#include "proto/parameters.pb.h"
#include "proto/sensor_msg.pb.h"
#include "software/multithreading/first_in_first_out_threaded_observer.h"
#include "software/multithreading/subject.hpp"
#include "software/sensor_fusion/sensor_fusion.h"
#include "software/world/world.h"

class ThreadedSensorFusion
    : public Subject<World>,
      public FirstInFirstOutThreadedObserver<SensorProto>,
      public FirstInFirstOutThreadedObserver<TbotsProto::ThunderbotsConfig>
{
   public:
    explicit ThreadedSensorFusion(TbotsProto::SensorFusionConfig config);
    virtual ~ThreadedSensorFusion() = default;

   private:
    void onValueReceived(SensorProto sensor_msg) override;
    void onValueReceived(TbotsProto::ThunderbotsConfig config) override;

    SensorFusion sensor_fusion;
    TbotsProto::SensorFusionConfig sensor_fusion_config;
    static constexpr size_t DIFFERENT_GRSIM_FRAMES_RECEIVED = 4;
    std::mutex sensor_fusion_mutex;
};
