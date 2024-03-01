#include "software/ai/hl/stp/strategy/strategy_impl.h"

#include "software/ai/evaluation/calc_best_shot.h"

StrategyImpl::StrategyImpl(const TbotsProto::AiConfig& ai_config, const Field& field)
    : field_(field)
{
    LOG(DEBUG) << "StrategyImpl boot up!";
    updateAiConfig(ai_config);
}

PossessionStrategy StrategyImpl::getPossessionStrategy(int num_robots)
{
    int unassigned_robots   = num_robots;
    int num_ideal_defenders = calcNumIdealDefenders();

    PossessionStrategy possession_strategy;

    possession_strategy.has_attacker = false;

    if (world_ptr_->getTeamWithPossession() == TeamPossession::FRIENDLY_TEAM)
    {
        possession_strategy.has_attacker = true;
        unassigned_robots -= 1;

        possession_strategy.num_defenders =
            std::min(num_ideal_defenders, std::max(unassigned_robots, 0));
        unassigned_robots -= possession_strategy.num_defenders;

        possession_strategy.num_support = unassigned_robots;

        return possession_strategy;
    }

    possession_strategy.num_defenders = std::min(unassigned_robots, num_ideal_defenders);
    unassigned_robots -= possession_strategy.num_defenders;

    if (unassigned_robots > 0)
    {
        possession_strategy.num_support = 1;
        unassigned_robots -= 1;
    }

    possession_strategy.num_defenders += unassigned_robots;

    return possession_strategy;
}

Pose StrategyImpl::getBestDribblePose(const Robot& robot)
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

PassWithRating StrategyImpl::getBestPass()
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

    return cached_pass_eval_->getBestPassOnField();
}

std::optional<Shot> StrategyImpl::getBestShot(const Robot& robot)
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

const TbotsProto::AiConfig& StrategyImpl::getAiConfig() const
{
    return ai_config_;
}

void StrategyImpl::updateAiConfig(const TbotsProto::AiConfig& ai_config)
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

bool StrategyImpl::hasWorld() const
{
    return world_ptr_ != nullptr;
}

void StrategyImpl::updateWorld(const WorldPtr& world_ptr)
{
    world_ptr_ = world_ptr;
    pass_strategy_->updateWorld(world_ptr_);
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

int StrategyImpl::calcNumIdealDefenders()
{
    // TODO(arun): make a todo
    return 2;
}
