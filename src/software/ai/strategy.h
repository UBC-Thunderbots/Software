#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/shot.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/receiver_position_generator.hpp"
#include "software/ai/passing/sampling_pass_generator.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/pose.h"
#include "software/world/field.h"

/**
 * Strategy contains shared gameplay-related calculations.
 *
 * By indirectly calling gameplay evaluation functions via Strategy, we can cache
 * their results in the Strategy class to avoid expensive recalculations.
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
     * Gets the best pass on the field.
     * 
     * @return the best pass
     */
    PassWithRating getBestPass();

    /**
     * Gets the next committed pass that has not yet been returned by this method
     * since the last World update.
     * 
     * @return the next committed pass, or std::nullopt if there are no more 
     * committed passes to return
     */
    std::optional<Pass> getNextCommittedPass();

    /**
     * Commits a pass.
     * 
     * @param pass the pass to commit
     */
    void commitPass(Pass pass);
    
    /**
     * Gets the next best receiving position on the field that has not yet been 
     * returned by this method since the last World update.
     * 
     * @return the next best receiving position
     */
    Point getNextBestReceivingPosition();

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

    WorldPtr world_ptr_;

    SamplingPassGenerator sampling_pass_generator_;
    ReceiverPositionGenerator<EighteenZoneId> receiver_position_generator_;

    std::optional<PassWithRating> best_pass_;
    std::vector<Pass> committed_passes_;
    size_t committed_passes_index_;

    std::vector<Point> receiving_positions_;
    size_t receiving_positions_index_;

    std::unordered_map<RobotId, std::optional<Shot>> robot_to_best_shot_;
};
