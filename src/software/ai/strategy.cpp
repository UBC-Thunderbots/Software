#include "software/ai/strategy.h"

Strategy::Strategy(const TbotsProto::AiConfig& ai_config)
    : ai_config_(ai_config),
      sampling_pass_generator_(ai_config_.passing_config()),
      receiver_position_generator_(
          std::make_shared<EighteenZonePitchDivision>(Field::createSSLDivisionBField()),
          ai_config_.passing_config())
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

PassWithRating Strategy::getBestPass()
{
    if (!best_pass_) 
    {
        best_pass_ = sampling_pass_generator_.getBestPass(*world_ptr_); 
    }

    return sampling_pass_generator_.getBestPass(*world_ptr_);
}

std::vector<Point> Strategy::getBestReceivingPositions()
{
    if (receiver_positions_.empty()) 
    {
        receiver_positions_ = receiver_position_generator_.getBestReceivingPositions(
            *world_ptr_, std::max(0, static_cast<int>(world_ptr_->friendlyTeam().numRobots()) - 1), 
            {getBestPass().pass.receiverPoint()});
    }

    return receiver_positions_;
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

    // Pass generators must be recreated with the new passing config
    sampling_pass_generator_ = SamplingPassGenerator(ai_config_.passing_config());
    receiver_position_generator_ = ReceiverPositionGenerator<EighteenZoneId>(
          std::make_shared<EighteenZonePitchDivision>(Field::createSSLDivisionBField()),
          ai_config_.passing_config());
}

void Strategy::updateWorld(const WorldPtr& world_ptr)
{
    world_ptr_ = world_ptr;

    best_pass_.reset();
    receiver_positions_.clear();
    robot_to_best_shot_.clear();
}
