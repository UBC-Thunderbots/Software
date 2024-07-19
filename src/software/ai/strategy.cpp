#include "software/ai/strategy.h"

Strategy::Strategy(const TbotsProto::AiConfig& ai_config)
    : ai_config_(ai_config),
      pass_generator_(ai_config_.passing_config()),
      receiver_position_generator_(
          std::make_shared<EighteenZonePitchDivision>(Field::createSSLDivisionBField()),
          ai_config_.passing_config())
{
}

PassWithRating Strategy::getBestPass(const std::vector<RobotId>& robots_to_ignore)
{
    return pass_generator_.getBestPass(*world_ptr_, robots_to_ignore);
}

std::vector<Point> Strategy::getBestReceivingPositions(
    unsigned int num_positions, const std::vector<Point>& existing_receiver_positions,
    const std::optional<Point>& pass_origin_override)
{
    return receiver_position_generator_.getBestReceivingPositions(
        *world_ptr_, num_positions, existing_receiver_positions, pass_origin_override);
}

std::optional<Shot> Strategy::getBestShot(const Robot& robot)
{
    return calcBestShotOnGoal(world_ptr_->field(), world_ptr_->friendlyTeam(),
                              world_ptr_->enemyTeam(), world_ptr_->ball().position(),
                              TeamType::ENEMY, {robot});
}

std::optional<Shot> Strategy::getBestSampledShot(const Robot& robot)
{
    return sampleForBestShotOnGoal(
        world_ptr_->field(), world_ptr_->friendlyTeam(), world_ptr_->enemyTeam(),
        world_ptr_->ball().position(), TeamType::ENEMY,
        ai_config_.dribble_config().max_continuous_dribbling_distance(),
        ai_config_.shot_config().num_shot_origin_points_to_sample(), {robot});
}

const TbotsProto::AiConfig& Strategy::getAiConfig() const
{
    return ai_config_;
}

void Strategy::updateAiConfig(const TbotsProto::AiConfig& ai_config)
{
    ai_config_ = ai_config;

    // Pass generators must be recreated with the new passing config
    pass_generator_              = PassGenerator(ai_config_.passing_config());
    receiver_position_generator_ = ReceiverPositionGenerator<EighteenZoneId>(
        std::make_shared<EighteenZonePitchDivision>(Field::createSSLDivisionBField()),
        ai_config_.passing_config());
}

void Strategy::updateWorld(const WorldPtr& world_ptr)
{
    world_ptr_ = world_ptr;
}
