#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/shot.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.h"
#include "software/ai/passing/receiver_position_generator.hpp"
#include "software/geom/algorithms/distance.h"
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
     * Gets the best receiving positions for the friendly robots to go to.
     *
     * @see ReceiverPositionGenerator::getBestReceivingPositions
     */
    std::vector<Point> getBestReceivingPositions(
        unsigned int num_positions,
        const std::vector<Point>& existing_receiver_positions = {},
        const std::optional<Point>& pass_origin_override      = std::nullopt);

    /**
     * Gets the best shot on goal for the given robot.
     *
     * @param robot the robot to find the best shot for
     *
     * @returns the best shot on goal, if one exists
     */
    std::optional<Shot> getBestShot(const Robot& robot);

    /**
     * Gets the best shot on goal by sampling multiple potential shot origin
     * points for the given robot.
     *
     * @param robot the robot to find the best shot for
     *
     * @returns the best shot on goal found by sampling, if one exists
     */
    std::optional<Shot> getBestSampledShot(const Robot& robot);

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

    PassGenerator pass_generator_;
    ReceiverPositionGenerator<EighteenZoneId> receiver_position_generator_;

    std::optional<PassWithRating> best_pass_;

    std::unordered_map<RobotId, std::optional<Shot>> robot_to_best_shot_;
    std::unordered_map<RobotId, std::optional<Shot>> robot_to_best_sampled_shot_;
};
