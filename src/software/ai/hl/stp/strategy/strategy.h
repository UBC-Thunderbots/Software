#pragma once

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/tactic/offense_support_tactic.h"
#include "software/ai/passing/pass.h"
#include "software/geom/pose.h"
#include "software/world/robot.h"
#include "software/world/robot_state.h"

/**
 * Contains shared gameplay-related calculations.
 */
class Strategy
{
   public:
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
    Pass getBestPass(const Robot& robot);

    std::optional<Shot> getBestShot(const Robot& robot, const World& world);

    std::vector<OffenseSupportType> getCommittedOffenseSupport() const;

    /**
     * Reset internal strategy calculations.
     */
    void reset();

   private:
    std::unordered_map<RobotId, Pose> robot_to_best_dribble_location_;
    std::unordered_map<RobotId, Pass> robot_to_best_pass_;
    std::unordered_map<RobotId, std::optional<Shot>> robot_to_best_shot_;
};
