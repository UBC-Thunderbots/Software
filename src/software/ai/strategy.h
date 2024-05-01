#pragma once

#include "proto/parameters.pb.h"
#include "proto/strategy.pb.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/shot.h"
#include "software/ai/passing/threaded_pass_generator.hpp"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/pose.h"
#include "software/world/field.h"

/**
 * Strategy contains shared gameplay-related calculations.
 * 
 * By indirectly calling gameplay evaluation functions via Strategy, we can cache 
 * their results in the Strategy class to avoid repeated expensive recalculations.
 */
class Strategy
{
   public:
    /**
     * Create a Strategy.
     * 
     * @param ai_config the AI configuration
     */
    Strategy(const TbotsProto::AiConfig& ai_config);

    /**
     * Get the possession strategy detailing the number of attackers,
     * supporters, and defenders to assign.
     */
    TbotsProto::PossessionStrategy getPossessionStrategy(int num_robots);

    /**
     * Get the best dribble pose for the given robot.
     *
     * @param robot robot to find best dribble location for
     *
     * @returns the best dribble pose
     */
    Pose getBestDribblePose(const Robot& robot);

    /**
     * Gets the best uncommitted pass on the entire field.
     * 
     * Uncommitted passes are guaranteed to be located in a different zone 
     * than any of the currently committed passes.
     *
     * @returns the best uncommitted pass, if one exists
     */
    std::optional<PassWithRating> getBestUncommittedPass();
    
    /**
     * Gets the best committed pass.
     *
     * @returns the best committed pass, if one exists
     */
    std::optional<PassWithRating> getBestCommittedPass();

    /**
     * Commits a pass, designating it as having a robot assigned to its
     * receiver point. 
     * 
     * @param pass the pass to commit
     */
    void commitPass(const PassWithRating& pass);

    /**
     * Gets the best shot on goal for the given robot.
     * 
     * @param robot the robot to find the best shot for
     * 
     * @returns the best shot on goal, if one exists
     */
    std::optional<Shot> getBestShot(const Robot& robot);

    /**
     * Gets the current AI configuration in use.
     * 
     * @returns the current AI configuration
     */
    const TbotsProto::AiConfig& getAiConfig() const;

    /**
     * Updates the current AI configuration.
     * 
     * @param ai_config the new AI configuration to use
     */
    void updateAiConfig(const TbotsProto::AiConfig& ai_config);
    
    /**
     * Updates the current World to use in gameplay calculations.
     * 
     * @param world_ptr the new World
     */
    void updateWorld(const WorldPtr& world_ptr);

   private:
    TbotsProto::AiConfig ai_config_;

    // World
    WorldPtr world_ptr_;
    std::shared_ptr<EighteenZonePitchDivision> pitch_division_;

    // Passing 
    std::unique_ptr<ThreadedPassGenerator<EighteenZoneId>> pass_generator_;
    std::shared_ptr<PassEvaluation<EighteenZoneId>> cached_pass_eval_;
    Timestamp cached_pass_time_;

    std::vector<EighteenZoneId> cached_ranked_pass_zones_;
    std::vector<PassWithRating> committed_passes_;

    std::unordered_map<RobotId, Pose> robot_to_best_dribble_location_;
    std::unordered_map<RobotId, std::optional<Shot>> robot_to_best_shot_;
};
