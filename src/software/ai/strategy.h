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
 */
class Strategy
{
   public:
    /**
     * Create a Strategy with an initial AI configuration.
     *
     * @param ai_config the AI configuration
     */
    explicit Strategy(const TbotsProto::AiConfig& ai_config);

    /**
     * Gets the best pass on the field.
     *
     * @see PassGenerator::getBestPass
     * 
     * @param world The state of the world
     * @param robots_to_ignore A list of robot ids to ignore when generating passes
     *
     * @return The best pass that can be made and its rating
     */
    PassWithRating getBestPass(const World& world,
                               const std::vector<RobotId>& robots_to_ignore = {});

    /**
     * Gets the best receiving positions for the friendly robots to go to.
     *
     * @see ReceiverPositionGenerator::getBestReceivingPositions
     * 
     * @param world The world to generate the best receiving positions based on
     * @param num_positions The number of receiving positions to generate
     * @param existing_receiver_positions A set of existing receiver positions that will
     * be avoided, if possible, when generating the new receiver positions.
     * @param pass_origin_override An optional override for the position where the pass
     * will be made from. If not provided, the ball position will be used. This could be
     * helpful if you are trying to position the receivers based on the ball's future
     * position.
     * 
     * @return A vector of
     *      min(num_positions, num_friendly_robots - existing_receiver_positions.size())
     * positions that the receivers could use.
     */
    std::vector<Point> getBestReceivingPositions(
        const World& world, unsigned int num_positions,
        const std::vector<Point>& existing_receiver_positions = {},
        const std::optional<Point>& pass_origin_override      = std::nullopt);

    /**
     * Gets the best shot on goal for the given robot.
     *
     * @param world the state of the World
     * @param robot the robot to find the best shot for
     *
     * @returns the best shot on goal, if one exists
     */
    std::optional<Shot> getBestShot(const World& world, const Robot& robot);

    /**
     * Gets the best shot on goal by sampling multiple potential shot origin
     * points for the given robot.
     *
     * @param world the state of the World
     * @param robot the robot to find the best shot for
     *
     * @returns the best shot on goal found by sampling, if one exists
     */
    std::optional<Shot> getBestSampledShot(const World& world, const Robot& robot);

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

   private:
    TbotsProto::AiConfig ai_config_;

    PassGenerator pass_generator_;
    ReceiverPositionGenerator<EighteenZoneId> receiver_position_generator_;
};
