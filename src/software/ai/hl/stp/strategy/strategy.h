#pragma once

#include <condition_variable>
#include "proto/parameters.pb.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/tactic/offense_support_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_generator.hpp"
#include "software/geom/pose.h"
#include "software/world/robot.h"
#include "software/world/robot_state.h"

/**
 * Contains shared gameplay-related calculations.
 */
class Strategy
{
   public:
    Strategy(const TbotsProto::AiConfig& ai_config);

    ~Strategy();

    void evaluatePassOptions();

    /**
     * Get the best dribble pose for the given robot
     *
     * @param robot robot to find best dribble location for
     *
     * @returns best dribble pose
     */
    Pose getBestDribblePose(const Robot& robot);

    /**
     * Get the best pass for the given robot.
     *
     * @param robot robot to find the best pass for
     *
     * @returns best pass for the robot
     */
    PassWithRating getBestPass();

    std::optional<Shot> getBestShot(const Robot& robot);

    std::vector<OffenseSupportType> getCommittedOffenseSupport() const;

    /**
     * Reset internal strategy calculations.
     */
    void reset();

    void updateAiConfig(const TbotsProto::AiConfig& ai_config);

    void updateWorld(const World& world);

   private:
    bool isBetterPassThanCached(const Timestamp& timestamp, const PassWithRating& pass);

    TbotsProto::AiConfig ai_config_;

    // World
    std::condition_variable world_available_cv_;
    std::mutex world_lock_;
    std::optional<const World> current_world_;

    // Passing calculations
    std::thread passing_thread_;
    std::mutex pass_generator_lock_;
    std::unique_ptr<PassGenerator<EighteenZoneId>> pass_generator_;
    std::condition_variable pass_available_cv_;
    std::shared_ptr<PassEvaluation<EighteenZoneId>> latest_pass_eval_;
    std::shared_ptr<PassEvaluation<EighteenZoneId>> cached_pass_eval_;
    Timestamp cached_pass_time_;

    std::atomic<bool> end_analysis_;

    std::unordered_map<RobotId, Pose> robot_to_best_dribble_location_;
    std::unordered_map<RobotId, std::optional<Shot>> robot_to_best_shot_;
};
