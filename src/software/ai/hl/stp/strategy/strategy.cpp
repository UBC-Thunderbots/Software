#include "software/ai/hl/stp/strategy/strategy.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.hpp"
#include "software/ai/passing/pass_with_rating.h"
#include "software/time/time.h"

Strategy::Strategy(const TbotsProto::AiConfig& ai_config)
    : end_analysis_(false)
{
    updateAiConfig(ai_config);

    passing_thread_ = std::thread(&Strategy::evaluatePassOptions, this);
}

Strategy::~Strategy()
{
    end_analysis_ = true;

    passing_thread_.join();
}

void Strategy::evaluatePassOptions()
{
    {
        std::unique_lock<std::mutex> lock(world_lock_);
        world_available_cv_.wait(lock, [&] { return current_world_.has_value(); });
    }

    while (!end_analysis_)
    {
        std::scoped_lock lock(pass_generator_lock_, world_lock_);
        latest_pass_eval_ = std::make_shared<PassEvaluation<EighteenZoneId>>(
                pass_generator_->generatePassEvaluation(current_world_.value()));
        pass_available_cv_.notify_one();
    }
}

Pose Strategy::getBestDribblePose(const Robot& robot)
{
    CHECK(current_world_.has_value()) << "[Strategy] Cannot generate next Stategy without latest world!";

    if (robot_to_best_dribble_location_.contains(robot.id()))
    {
        return robot_to_best_dribble_location_.at(robot.id());
    }

    // TODO(#3082): temporary logic, find best dribble_position
    std::lock_guard<std::mutex> lock(world_lock_);
    CHECK(current_world_.has_value()) << "[Strategy] Cannot generate Strategy without altest world!";
    const World& world = current_world_.value();
    Vector robot_to_goal = world.field().enemyGoalCenter() - robot.position();
    Point dribble_position = robot.position() + robot_to_goal.normalize(0.8);

    // cache the dribble position
    robot_to_best_dribble_location_[robot.id()] = Pose(dribble_position, robot_to_goal.orientation());

    return robot_to_best_dribble_location_.at(robot.id());
}

PassWithRating Strategy::getBestPass()
{
    // wait for a pass evaluation to be available
    {
        std::unique_lock<std::mutex> lock(pass_generator_lock_);
        pass_available_cv_.wait(lock, [&] { return (latest_pass_eval_ != nullptr); });
    }

    // calculate best pass

    Timestamp current_time;
    {
        std::lock_guard<std::mutex> world_lock(world_lock_);
        CHECK(current_world_.has_value()) << "[Strategy] Cannot generate Strategy without latest world!";
        current_time = current_world_.value().getMostRecentTimestamp();
    }

    std::lock_guard<std::mutex> pass_gen_lock(pass_generator_lock_);
    const auto& latest_pass = latest_pass_eval_->getBestPassOnField();
    if (isBetterPassThanCached(current_time, latest_pass))
    {
        cached_pass_eval_ = latest_pass_eval_;
        cached_pass_time_ = current_time;
    }

    return cached_pass_eval_->getBestPassOnField();
}

std::optional<Shot> Strategy::getBestShot(const Robot& robot)
{
    if (robot_to_best_shot_.contains(robot.id()))
    {
        return robot_to_best_shot_.at(robot.id());
    }

    std::lock_guard<std::mutex> world_lock(world_lock_);
    CHECK(current_world_.has_value()) << "[Strategy] Cannot generate Strategy without latest world!";
    const World& world = current_world_.value();
    robot_to_best_shot_[robot.id()] =
        calcBestShotOnGoal(world.field(), world.friendlyTeam(), world.enemyTeam(),
                           robot.position(), TeamType::ENEMY, {robot});
    return robot_to_best_shot_[robot.id()];
}

std::vector<OffenseSupportType> Strategy::getCommittedOffenseSupport() const
{
    // TODO(#3098): Commit the Support tactics to this Strategy class and return their
    // types here
    return std::vector<OffenseSupportType>();
}

void Strategy::reset()
{
    robot_to_best_dribble_location_ = {};
    robot_to_best_shot_             = {};
}

void Strategy::updateAiConfig(const TbotsProto::AiConfig& ai_config)
{
    ai_config_ = ai_config;
    
    {
        std::lock_guard<std::mutex> lock(pass_generator_lock_);

        pass_generator_ = std::make_unique<PassGenerator<EighteenZoneId>>(
            std::make_shared<const EighteenZonePitchDivision>(Field::createSSLDivisionBField()),
            ai_config.passing_config());
    }

    reset();
}

void Strategy::updateWorld(const World& world)
{
    const std::lock_guard<std::mutex> lock(world_lock_);
    current_world_.emplace(world);
    world_available_cv_.notify_one();
}

bool Strategy::isBetterPassThanCached(const Timestamp& timestamp, const PassWithRating& pass)
{
    bool is_cache_time_expired = (timestamp - cached_pass_time_)
        < Duration::fromSeconds(ai_config_.passing_config().pass_recalculation_commit_time_s());
    bool is_cache_pass_better = (cached_pass_eval_ != nullptr)
        && cached_pass_eval_->getBestPassOnField().rating > pass.rating;

    return is_cache_time_expired || !is_cache_pass_better;
}
