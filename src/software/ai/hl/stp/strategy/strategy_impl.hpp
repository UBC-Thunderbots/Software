#pragma once

#include "proto/parameters.pb.h"
#include "proto/strategy.pb.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/strategy/pass_strategy.h"
#include "software/ai/hl/stp/tactic/offense_support_tactics/offense_support_type.h"
#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_evaluation.hpp"
#include "software/ai/passing/pass_with_rating.h"
#include "software/geom/pose.h"
#include "software/world/field.h"

/**
 * Contains shared gameplay-related calculations.
 */
template <class PitchDivision, class ZoneEnum>
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
    Pass getBestCommittedPass();

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
    void commit(OffenseSupportType type);
    std::vector<OffenseSupportType> getCommittedOffenseSupport() const;

    void commit(const Pass& pass);

   private:
    bool isBetterPassThanCached(const Timestamp& timestamp, const PassWithRating& pass);

    int calcNumIdealDefenders();

    TbotsProto::AiConfig ai_config_;

    // World
    Field field_;
    WorldPtr world_ptr_;
    PitchDivision pitch_division_;

    // Passing
    std::unique_ptr<PassStrategy> pass_strategy_;
    std::shared_ptr<PassEvaluation<ZoneEnum>> cached_pass_eval_;
    Timestamp cached_pass_time_;
    std::vector<Pass> committed_passes_;

    std::optional<Shot> best_shot_;

    std::vector<OffenseSupportType> committed_support_types_;
    std::unordered_map<RobotId, Pose> robot_to_best_dribble_location_;
    std::unordered_map<RobotId, std::optional<Shot>> robot_to_best_shot_;
};

template <class PitchDivision, class ZoneEnum>
StrategyImpl<PitchDivision, ZoneEnum>::StrategyImpl(const TbotsProto::AiConfig& ai_config,
                                                    const Field& field)
    : field_(field), pitch_division_(field)
{
    updateAiConfig(ai_config);
}

template <class PitchDivision, class ZoneEnum>
TbotsProto::PossessionStrategy
StrategyImpl<PitchDivision, ZoneEnum>::getPossessionStrategy(int num_robots)
{
    TbotsProto::PossessionStrategy possession_strategy;

    int unassigned_robots   = num_robots;
    int num_ideal_defenders = calcNumIdealDefenders();

    if (world_ptr_->getTeamWithPossession() == TeamPossession::FRIENDLY_TEAM)
    {
        possession_strategy.set_attackers(1);
        unassigned_robots -= 1;

        possession_strategy.set_defenders(
            std::min(num_ideal_defenders, std::max(unassigned_robots, 0)));
        unassigned_robots -= possession_strategy.defenders();

        possession_strategy.set_supporters(unassigned_robots);

        return possession_strategy;
    }

    possession_strategy.set_defenders(std::min(num_ideal_defenders, unassigned_robots));
    unassigned_robots -= possession_strategy.defenders();

    possession_strategy.set_supporters(std::max(unassigned_robots, 0));

    return possession_strategy;
}

template <class PitchDivision, class ZoneEnum>
Pose StrategyImpl<PitchDivision, ZoneEnum>::getBestDribblePose(const Robot& robot)
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

template <class PitchDivision, class ZoneEnum>
PassWithRating StrategyImpl<PitchDivision, ZoneEnum>::getBestUncommittedPass()
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

    for (const auto& zone : cached_pass_eval_->rankZonesForReceiving(
             world_ptr_, world_ptr_->ball().position()))
    {
        if (std::find_if(committed_passes_.begin(), committed_passes_.end(),
                         [&](const Pass& p) {
                             return pitch_division_.getZoneId(p.receiverPoint()) == zone;
                         }) == committed_passes_.end())
        {
            return cached_pass_eval_->getBestPassInZones({zone});
        }
    }

    CHECK(true) << "No Pass found? All Zones have a pass committed in them...";

    Pass default_pass =
        Pass(world_ptr_->ball().position(), world_ptr_->field().friendlyGoalCenter(),
             ai_config_.passing_config().min_pass_speed_m_per_s());
    return PassWithRating{pass : default_pass, rating : 0};
}

template <class PitchDivision, class ZoneEnum>
Pass StrategyImpl<PitchDivision, ZoneEnum>::getBestCommittedPass()
{
    if (committed_passes_.empty())
    {
        Pass pass = getBestUncommittedPass().pass;
        committed_passes_.push_back(pass);

        return pass;
    }

    return committed_passes_[0];
}

template <class PitchDivision, class ZoneEnum>
std::optional<Shot> StrategyImpl<PitchDivision, ZoneEnum>::getBestShot(const Robot& robot)
{
    if (!robot_to_best_shot_.contains(robot.id()))
    {
        robot_to_best_shot_[robot.id()] = sampleForBestShotOnGoal(
            world_ptr_->field(), world_ptr_->friendlyTeam(), world_ptr_->enemyTeam(),
            world_ptr_->ball().position(), TeamType::ENEMY, 
            ai_config_.dribble_config().max_continuous_dribbling_distance(),
            ai_config_.shot_config().num_shot_origin_points_to_sample(),
            {robot});
    }

    return robot_to_best_shot_.at(robot.id());
}

template <class PitchDivision, class ZoneEnum>
void StrategyImpl<PitchDivision, ZoneEnum>::reset()
{
    robot_to_best_dribble_location_.clear();
    robot_to_best_shot_.clear();
}

template <class PitchDivision, class ZoneEnum>
const TbotsProto::AiConfig& StrategyImpl<PitchDivision, ZoneEnum>::getAiConfig() const
{
    return ai_config_;
}

template <class PitchDivision, class ZoneEnum>
void StrategyImpl<PitchDivision, ZoneEnum>::updateAiConfig(
    const TbotsProto::AiConfig& ai_config)
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

template <class PitchDivision, class ZoneEnum>
bool StrategyImpl<PitchDivision, ZoneEnum>::hasWorld() const
{
    return world_ptr_ != nullptr;
}

template <class PitchDivision, class ZoneEnum>
void StrategyImpl<PitchDivision, ZoneEnum>::updateWorld(const WorldPtr& world_ptr)
{
    world_ptr_ = world_ptr;
    pass_strategy_->updateWorld(world_ptr_);
    reset();
}

template <class PitchDivision, class ZoneEnum>
bool StrategyImpl<PitchDivision, ZoneEnum>::isBetterPassThanCached(
    const Timestamp& timestamp, const PassWithRating& pass)
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

template <class PitchDivision, class ZoneEnum>
int StrategyImpl<PitchDivision, ZoneEnum>::calcNumIdealDefenders()
{
    // TODO(arun): make a todo
    return 2;
}

template <class PitchDivision, class ZoneEnum>
void StrategyImpl<PitchDivision, ZoneEnum>::commit(
    OffenseSupportType offense_support_type)
{
    committed_support_types_.push_back(offense_support_type);
}

template <class PitchDivision, class ZoneEnum>
void StrategyImpl<PitchDivision, ZoneEnum>::commit(const Pass& pass)
{
    committed_passes_.push_back(pass);
}
