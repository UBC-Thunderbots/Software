#pragma once

#include "proto/parameters.pb.h"
#include "proto/strategy.pb.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/strategy/pass_strategy.h"
#include "software/ai/hl/stp/tactic/offense_support_tactics/offense_support_type.h"
#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_evaluation.hpp"
#include "software/geom/pose.h"
#include "software/world/field.h"

/**
 * Contains shared gameplay-related calculations.
 */
template <class ZoneEnum>
class StrategyImpl
{
   public:
    StrategyImpl(const TbotsProto::AiConfig& ai_config,
                 const Field& field = Field::createSSLDivisionBField());

    TbotsProto::PossessionStrategy getPossessionStrategy(int num_robots);

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
    PassWithRating getBestUncommittedPass();

    std::optional<Shot> getBestShot(const Robot& robot);

    /**
     * Reset internal strategy calculations.
     */
    void reset();

    const TbotsProto::AiConfig& getAiConfig() const;
    void updateAiConfig(const TbotsProto::AiConfig& ai_config);

    bool hasWorld() const;
    void updateWorld(const WorldPtr& world_ptr);

    // Committed OffenseSupportTypes
    void commit(OffenseSupportType& type);
    std::vector<OffenseSupportType> getCommittedOffenseSupport() const;

    void commit(const Pass& pass);

   private:
    bool isBetterPassThanCached(const Timestamp& timestamp, const PassWithRating& pass);

    int calcNumIdealDefenders();

    TbotsProto::AiConfig ai_config_;

    // World
    Field field_;
    WorldPtr world_ptr_;

    // Passing
    std::unique_ptr<PassStrategy> pass_strategy_;
    std::shared_ptr<PassEvaluation<EighteenZoneId>> cached_pass_eval_;
    Timestamp cached_pass_time_;
    std::vector<ZoneEnum> committed_pass_zones_;

    std::vector<OffenseSupportType> committed_support_types_;
    std::unordered_map<RobotId, Pose> robot_to_best_dribble_location_;
    std::unordered_map<RobotId, std::optional<Shot>> robot_to_best_shot_;
};

template <class ZoneEnum>
StrategyImpl<ZoneEnum>::StrategyImpl(const TbotsProto::AiConfig& ai_config, const Field& field)
    : field_(field)
{
    LOG(DEBUG) << "StrategyImpl boot up!";
    updateAiConfig(ai_config);
}

template <class ZoneEnum>
TbotsProto::PossessionStrategy StrategyImpl<ZoneEnum>::getPossessionStrategy(int num_robots)
{
    TbotsProto::PossessionStrategy possession_strategy;

    int unassigned_robots   = num_robots;
    int num_ideal_defenders = calcNumIdealDefenders();

    if (world_ptr_->getTeamWithPossession() == TeamPossession::FRIENDLY_TEAM)
    {
        possession_strategy.set_attackers(1);
        unassigned_robots -= 1;

        possession_strategy.set_defenders(std::min(num_ideal_defenders, std::max(unassigned_robots, 0)));
        unassigned_robots -= possession_strategy.defenders();

        possession_strategy.set_supporters(unassigned_robots);

        return possession_strategy;
    }

    possession_strategy.set_defenders(std::min(num_ideal_defenders, unassigned_robots));
    unassigned_robots -= possession_strategy.defenders();

    possession_strategy.set_supporters(std::max(unassigned_robots, 0));

    return possession_strategy;
}

template <class ZoneEnum>
Pose StrategyImpl<ZoneEnum>::getBestDribblePose(const Robot& robot)
{
    if (robot_to_best_dribble_location_.contains(robot.id()))
    {
        return robot_to_best_dribble_location_.at(robot.id());
    }

    // TODO(#3082): temporary logic, find best dribble_position
    Vector robot_to_goal   = world_ptr_->field().enemyGoalCenter() - robot.position();
    Point dribble_position = robot.position() + robot_to_goal.normalize(0.8);

    // cache the dribble position
    robot_to_best_dribble_location_[robot.id()] =
        Pose(dribble_position, robot_to_goal.orientation());

    return robot_to_best_dribble_location_.at(robot.id());
}

template <class ZoneEnum>
PassWithRating StrategyImpl<ZoneEnum>::getBestUncommittedPass()
{
    // calculate best pass
    Timestamp current_time = world_ptr_->getMostRecentTimestamp();

    auto pass_eval          = pass_strategy_->getPassEvaluation();
    const auto& latest_pass = pass_eval->getBestPassOnField();
    if (isBetterPassThanCached(current_time, latest_pass))
    {
        cached_pass_eval_ = pass_eval;
        cached_pass_time_ = current_time;
    }

    for (const ZoneEnum& zone : cached_pass_eval_->rankZonesForReceiving(world_ptr_, world_ptr_->ball().position()))
    {
        if (std::find(committed_pass_zones_.begin(), committed_pass_zones_.end(), zone) == committed_pass_zones_.end())
        {
            return cached_pass_eval_->getBestPassInZones({zone});
        }
    }

    CHECK(true) << "No Pass found? All Zones have a pass committed in them...";

    PassWithRating default_pass;
    return default_pass;
}

template <class ZoneEnum>
std::optional<Shot> StrategyImpl<ZoneEnum>::getBestShot(const Robot& robot)
{
    if (robot_to_best_shot_.contains(robot.id()))
    {
        return robot_to_best_shot_.at(robot.id());
    }

    robot_to_best_shot_[robot.id()] = calcBestShotOnGoal(
        world_ptr_->field(), world_ptr_->friendlyTeam(), world_ptr_->enemyTeam(),
        robot.position(), TeamType::ENEMY, {robot});
    return robot_to_best_shot_[robot.id()];
}

template <class ZoneEnum>
std::vector<OffenseSupportType> StrategyImpl<ZoneEnum>::getCommittedOffenseSupport() const
{
    return committed_support_types_;
}

template <class ZoneEnum>
void StrategyImpl<ZoneEnum>::reset()
{
    robot_to_best_dribble_location_ = {};
    robot_to_best_shot_             = {};
    committed_pass_zones_           = {};
    committed_support_types_        = {};
}

template <class ZoneEnum>
const TbotsProto::AiConfig& StrategyImpl<ZoneEnum>::getAiConfig() const
{
    return ai_config_;
}

template <class ZoneEnum>
void StrategyImpl<ZoneEnum>::updateAiConfig(const TbotsProto::AiConfig& ai_config)
{
    LOG(DEBUG) << "[Strategy] Updating AI config";
    ai_config_ = ai_config;

    pass_strategy_ = std::make_unique<PassStrategy>(ai_config.passing_config(), field_);
    if (world_ptr_)
    {
        pass_strategy_->updateWorld(world_ptr_);
    }

    reset();
}

template <class ZoneEnum>
bool StrategyImpl<ZoneEnum>::hasWorld() const
{
    return world_ptr_ != nullptr;
}

template <class ZoneEnum>
void StrategyImpl<ZoneEnum>::updateWorld(const WorldPtr& world_ptr)
{
    world_ptr_ = world_ptr;
    pass_strategy_->updateWorld(world_ptr_);
}

template <class ZoneEnum>
bool StrategyImpl<ZoneEnum>::isBetterPassThanCached(const Timestamp& timestamp,
                                          const PassWithRating& pass)
{
    bool is_cache_time_expired =
        (timestamp - cached_pass_time_) <
        Duration::fromSeconds(
            ai_config_.passing_config().pass_recalculation_commit_time_s());
    bool is_cache_pass_better =
        (cached_pass_eval_ != nullptr) &&
        cached_pass_eval_->getBestPassOnField().rating > pass.rating;

    return is_cache_time_expired || !is_cache_pass_better;
}

template <class ZoneEnum>
int StrategyImpl<ZoneEnum>::calcNumIdealDefenders()
{
    // TODO(arun): make a todo
    return 2;
}

template <class ZoneEnum>
void StrategyImpl<ZoneEnum>::commit(OffenseSupportType& offense_support_type)
{
    committed_support_types_.push_back(offense_support_type);
}

template <class ZoneEnum>
void StrategyImpl<ZoneEnum>::commit(const Pass& pass)
{
    committed_pass_zones_.push_back(ZoneEnum().getZoneId(pass.receiverPoint()));
}
