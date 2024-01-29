#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/strategy/pass_strategy.h"
#include "software/ai/hl/stp/tactic/offense_support_tactic.h"
#include "software/geom/pose.h"
#include "software/world/robot.h"
#include "software/world/robot_state.h"

class StrategyImpl;

class Strategy
{
   public:
    Strategy(const TbotsProto::AiConfig& ai_config,
             const Field& field = Field::createSSLDivisionBField());

    void updateAiConfig(const TbotsProto::AiConfig& ai_config);
    void updateWorld(const World& world);

    std::unique_ptr<StrategyImpl> operator->();

   private:
    std::unique_ptr<StrategyImpl> strategy_;
};


/**
 * Contains shared gameplay-related calculations.
 */
class StrategyImpl
{
  public:
    StrategyImpl(const TbotsProto::AiConfig& ai_config,
                 const Field& field = Field::createSSLDivisionBField());

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

    bool hasWorld() const;
    void updateWorld(const World& world);

   private:
    bool isBetterPassThanCached(const Timestamp& timestamp, const PassWithRating& pass);

    TbotsProto::AiConfig ai_config_;

    // World
    Field field_;
    std::optional<std::reference_wrapper<const World>> world_;

    // Passing
    std::unique_ptr<PassStrategy> pass_strategy_;
    std::shared_ptr<PassEvaluation<EighteenZoneId>> cached_pass_eval_;
    Timestamp cached_pass_time_;

    std::unordered_map<RobotId, Pose> robot_to_best_dribble_location_;
    std::unordered_map<RobotId, std::optional<Shot>> robot_to_best_shot_;

    friend class Strategy;
};

