#include "software/ai/hl/stp/strategy/strategy.h"

#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_evaluation.hpp"
#include "software/ai/passing/pass_with_rating.h"

Strategy::Strategy(const TbotsProto::AiConfig& ai_config, const Field& field)
    : strategy_(std::make_unique<StrategyImpl>(ai_config, field))
{
}

void Strategy::updateAiConfig(const TbotsProto::AiConfig& ai_config)
{
    strategy_->updateAiConfig(ai_config);
}

void Strategy::updateWorld(const World& world)
{
    strategy_->updateWorld(world);
}

std::unique_ptr<StrategyImpl> Strategy::operator->()
{
    CHECK(strategy_->hasWorld())
        << "[StrategyImpl] Cannot generate next Strategy without a World!";
    return strategy_;
}

StrategyImpl::StrategyImpl(const TbotsProto::AiConfig& ai_config, const Field& field)
    : field_(field),
      pass_strategy_(std::make_unique<PassStrategy>(ai_config.passing_config(), field))
{
    updateAiConfig(ai_config);
}

Pose StrategyImpl::getBestDribblePose(const Robot& robot)
{
    if (robot_to_best_dribble_location_.contains(robot.id()))
    {
        return robot_to_best_dribble_location_.at(robot.id());
    }

    // TODO(#3082): temporary logic, find best dribble_position
    const World& world     = world_.value();
    Vector robot_to_goal   = world.field().enemyGoalCenter() - robot.position();
    Point dribble_position = robot.position() + robot_to_goal.normalize(0.8);

    // cache the dribble position
    robot_to_best_dribble_location_[robot.id()] =
        Pose(dribble_position, robot_to_goal.orientation());

    return robot_to_best_dribble_location_.at(robot.id());
}

PassWithRating StrategyImpl::getBestPass()
{
    // calculate best pass
    Timestamp current_time;
    {
        const World& world = world_.value();
        current_time = world.getMostRecentTimestamp();
    }

    auto pass_eval = pass_strategy_->getPassEvaluation();
    const auto& latest_pass                  = pass_eval->getBestPassOnField();
    if (isBetterPassThanCached(current_time, latest_pass))
    {
        cached_pass_eval_ = pass_eval;
        cached_pass_time_ = current_time;
    }

    return cached_pass_eval_->getBestPassOnField();
}

std::optional<Shot> StrategyImpl::getBestShot(const Robot& robot)
{
    if (robot_to_best_shot_.contains(robot.id()))
    {
        return robot_to_best_shot_.at(robot.id());
    }

    const World& world = world_.value();
    robot_to_best_shot_[robot.id()] =
        calcBestShotOnGoal(world.field(), world.friendlyTeam(), world.enemyTeam(),
                           robot.position(), TeamType::ENEMY, {robot});
    return robot_to_best_shot_[robot.id()];
}

std::vector<OffenseSupportType> StrategyImpl::getCommittedOffenseSupport() const
{
    // TODO(#3098): Commit the Support tactics to this StrategyImpl class and return their
    // types here
    return std::vector<OffenseSupportType>();
}

void StrategyImpl::reset()
{
    robot_to_best_dribble_location_ = {};
    robot_to_best_shot_             = {};
}

void StrategyImpl::updateAiConfig(const TbotsProto::AiConfig& ai_config)
{
    ai_config_ = ai_config;

    pass_strategy_ = std::make_unique<PassStrategy>(ai_config.passing_config(), field_);
    if (world_)
    {
        pass_strategy_->updateWorld(static_cast<const World&>(world_.value()));
    }

    reset();
}

bool StrategyImpl::hasWorld() const
{
    return world_.has_value();
}

void StrategyImpl::updateWorld(const World& world)
{
    world_.emplace(world);
}

bool StrategyImpl::isBetterPassThanCached(const Timestamp& timestamp,
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
