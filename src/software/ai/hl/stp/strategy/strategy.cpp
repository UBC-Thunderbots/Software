#include "software/ai/hl/stp/strategy/strategy.h"

#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_evaluation.hpp"
#include "software/ai/passing/pass_generator.hpp"
#include "software/ai/passing/pass_with_rating.h"
#include "software/time/time.h"

Strategy::Strategy(const TbotsProto::AiConfig& ai_config, const Field& field)
    : strategy_(ai_config, field)
{
}

void Strategy::updateWorld(const World& world)
{
    strategy_->updateWorld(world);
}

std::unique_ptr<StrategyImpl> Strategy::operator->()
{
    CHECK(strategy_->getWorld() != std::nullopt)
        << "[StrategyImpl] Cannot generate next Strategy without a World!";
    return strategy_;
}

StrategyImpl::StrategyImpl(const TbotsProto::AiConfig& ai_config, const Field& field)
    : pass_strategy_(ai_config.passing_config(), field)
{
    updateAiConfig(ai_config);
}

Pose StrategyImpl::getBestDribblePose(const Robot& robot)
{
    CHECK(current_world_.has_value())
        << "[StrategyImpl] Cannot generate next Stategy without latest world!";

    if (robot_to_best_dribble_location_.contains(robot.id()))
    {
        return robot_to_best_dribble_location_.at(robot.id());
    }

    // TODO(#3082): temporary logic, find best dribble_position
    std::lock_guard<std::mutex> lock(world_lock_);
    CHECK(current_world_.has_value())
        << "[StrategyImpl] Cannot generate Strategy without altest world!";
    const World& world     = current_world_.value();
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
        current_time = current_world_.value().getMostRecentTimestamp();
    }

    PassEvaluation<EighteenZoneId> pass_eval = pass_strategy_.getPassEvaluation();
    const auto& latest_pass                  = pass_eval_->getBestPassOnField();
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

    std::lock_guard<std::mutex> world_lock(world_lock_);
    CHECK(current_world_.has_value())
        << "[StrategyImpl] Cannot generate Strategy without latest world!";
    const World& world = current_world_.value();
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

    {
        std::lock_guard<std::mutex> lock(pass_generator_lock_);

        pass_generator_ = std::make_unique<PassGenerator<EighteenZoneId>>(
            std::make_shared<const EighteenZonePitchDivision>(
                Field::createSSLDivisionBField()),
            ai_config.passing_config());
    }

    reset();
}

void StrategyImpl::updateWorld(const World& world)
{
    const std::lock_guard<std::mutex> lock(world_lock_);
    current_world_.emplace(world);
    world_available_cv_.notify_one();
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
