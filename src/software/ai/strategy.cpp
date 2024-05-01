#include "software/ai/strategy.h"

Strategy::Strategy(const TbotsProto::AiConfig& ai_config) 
    : ai_config_(ai_config), 
      pitch_division_(std::make_shared<EighteenZonePitchDivision>(
        Field::createSSLDivisionBField())),
      pass_generator_(std::make_unique<ThreadedPassGenerator<EighteenZoneId>>(
        pitch_division_, ai_config_.passing_config()))
{
}

TbotsProto::PossessionStrategy Strategy::getPossessionStrategy(int num_robots)
{
    TbotsProto::PossessionStrategy possession_strategy;

    int unassigned_robots   = num_robots;
    int num_ideal_defenders = 2;

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

Pose Strategy::getBestDribblePose(const Robot& robot)
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

std::optional<PassWithRating> Strategy::getBestUncommittedPass()
{
    Timestamp current_time = world_ptr_->getMostRecentTimestamp();

    if (!cached_pass_eval_ ||
        (current_time - cached_pass_time_) >
            Duration::fromSeconds(
                ai_config_.passing_config().pass_recalculation_commit_time_s()))
    {
        cached_pass_time_         = current_time;
        cached_pass_eval_         = pass_generator_->getPassEvaluation();
        cached_ranked_pass_zones_ = cached_pass_eval_->rankZonesForReceiving(
            *world_ptr_, world_ptr_->ball().position());
    }

    for (EighteenZoneId zone : cached_ranked_pass_zones_)
    {
        // Predicate that filters out passes received inside the zone
        const auto is_pass_outside_zone = [&](const PassWithRating& pass) {
            return pitch_division_->getZoneId(pass.pass.receiverPoint()) != zone;
        };

        if (std::all_of(committed_passes_.begin(), committed_passes_.end(),
                        is_pass_outside_zone))
        {
            return cached_pass_eval_->getBestPassInZones({zone});
        }
    }

    return std::nullopt;
}

std::optional<PassWithRating> Strategy::getBestCommittedPass()
{
    if (committed_passes_.empty())
    {
        return std::nullopt;
    }

    // Predicate that filters out passes that are too short
    auto pass_filter = [&](const PassWithRating& pass_with_rating) {
        return distance(world_ptr_->ball().position(),
                        pass_with_rating.pass.receiverPoint()) >=
               ai_config_.passing_config().min_pass_distance();
    };

    std::vector<PassWithRating> filtered_passes;
    std::copy_if(committed_passes_.begin(), committed_passes_.end(),
                 std::back_inserter(filtered_passes), pass_filter);

    return *std::max_element(committed_passes_.begin(), committed_passes_.end());
}

void Strategy::commitPass(const PassWithRating& pass)
{
    committed_passes_.push_back(pass);
}

std::optional<Shot> Strategy::getBestShot(const Robot& robot)
{
    if (!robot_to_best_shot_.contains(robot.id()))
    {
        robot_to_best_shot_[robot.id()] = sampleForBestShotOnGoal(
            world_ptr_->field(), world_ptr_->friendlyTeam(), world_ptr_->enemyTeam(),
            world_ptr_->ball().position(), TeamType::ENEMY,
            ai_config_.dribble_config().max_continuous_dribbling_distance(),
            ai_config_.shot_config().num_shot_origin_points_to_sample(), {robot});
    }

    return robot_to_best_shot_.at(robot.id());
}

const TbotsProto::AiConfig& Strategy::getAiConfig() const
{
    return ai_config_;
}

void Strategy::updateAiConfig(const TbotsProto::AiConfig& ai_config)
{
    ai_config_ = ai_config;

    // The pass generator must be recreated with the new passing config
    pass_generator_ = std::make_unique<ThreadedPassGenerator<EighteenZoneId>>(
        pitch_division_, ai_config_.passing_config());
}

void Strategy::updateWorld(const WorldPtr& world_ptr)
{
    world_ptr_ = world_ptr;
    
    pass_generator_->updateWorld(world_ptr_);

    committed_passes_.clear();
    robot_to_best_dribble_location_.clear();
    robot_to_best_shot_.clear();
}
