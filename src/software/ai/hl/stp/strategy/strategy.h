#pragma once

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

    void evaluatePassOptions();

    /**
     * Get the best dribble pose for the given robot
     *
     * @param robot robot to find best dribble location for
     *
     * @returns best dribble pose
     */
    Pose getBestDribblePose(const Robot& robot, const World& world);

    /**
     * Get the best pass for the given robot.
     *
     * @param robot robot to find the best pass for
     *
     * @returns best pass for the robot
     */
    std::optional<Pass> getBestPass();

    std::optional<Shot> getBestShot(const Robot& robot, const World& world);

    std::vector<OffenseSupportType> getCommittedOffenseSupport() const;

    /**
     * Reset internal strategy calculations.
     */
    void reset();

    void updateAiConfig(const TbotsProto::AiConfig& ai_config);

    void updateWorld(const World& world);

   private:
    TbotsProto::AiConfig ai_config_;

    // World
    std::mutex world_lock_;
    std::optional<std::reference_wrapper<const World>> current_world_;

    // Passing thread
    std::thread passing_thread_;
    std::mutex pass_generator_lock_;
    std::unique_ptr<PassGenerator<EighteenZoneId>> pass_generator_;

    std::unordered_map<RobotId, Pose> robot_to_best_dribble_location_;
    std::unordered_map<RobotId, Pass> robot_to_best_pass_;
    std::unordered_map<RobotId, std::optional<Shot>> robot_to_best_shot_;
};
