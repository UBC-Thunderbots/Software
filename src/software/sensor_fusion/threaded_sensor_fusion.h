#pragma once

#include "proto/sensor_msg.pb.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/multithreading/first_in_first_out_threaded_observer.h"
#include "software/multithreading/subject.hpp"
#include "software/sensor_fusion/sensor_fusion.h"
#include "software/world/world.h"
//#include "software/world/game_state.h"

class ThreadedSensorFusion : public Subject<World>,
                             public FirstInFirstOutThreadedObserver<SensorProto>
{
   public:
    explicit ThreadedSensorFusion(
        std::shared_ptr<const SensorFusionConfig> sensor_fusion_config);
    virtual ~ThreadedSensorFusion() = default;

    //TODO This function should never see the light of day
    //   if you see this in a PR, I've messed up, badly
    //   you need to be protected from this horrible unsafe code
    void updateGameState(const GameState &game_state);

   private:
    void onValueReceived(SensorProto sensor_msg) override;

    SensorFusion sensor_fusion;
    static constexpr size_t DIFFERENT_GRSIM_FRAMES_RECEIVED = 4;
};
