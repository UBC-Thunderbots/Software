#pragma once

#include <condition_variable>
#include <thread>

#include "proto/parameters.pb.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_evaluation.hpp"
#include "software/ai/passing/pass_generator.hpp"
#include "software/world/world.h"

class PassStrategy
{
   public:
    PassStrategy(const TbotsProto::PassingConfig& passing_config, const Field& field);

    ~PassStrategy();

    PassEvaluation<EighteenZoneId> getPassEvaluation();

    void updateWorld(const World& world);

   private:
    void evaluatePassOptions();

    // World
    std::condition_variable world_available_cv_;
    std::mutex world_lock_;
    std::optional<const World> current_world_;

    // Passing calculations
    std::condition_variable pass_available_cv_;
    std::mutex pass_evaluation_lock_;
    PassGenerator<EighteenZoneId> pass_generator_;
    std::thread passing_thread_;
    std::unique_ptr<PassEvaluation<EighteenZoneId>> latest_pass_eval_;

    std::atomic<bool> end_analysis_;
};
