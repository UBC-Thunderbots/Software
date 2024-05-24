#include "software/ai/strategy.h"

Strategy::Strategy(const TbotsProto::AiConfig& ai_config)
    : ai_config_(ai_config),
      sampling_pass_generator_(ai_config_.passing_config()),
      receiver_position_generator_(
          std::make_shared<EighteenZonePitchDivision>(Field::createSSLDivisionBField()),
          ai_config_.passing_config())
{
}

PassWithRating Strategy::getBestPass()
{
    if (!best_pass_)
    {
        best_pass_ = sampling_pass_generator_.getBestPass(*world_ptr_);
    }

    return *best_pass_;
}

std::optional<Pass> Strategy::getNextCommittedPass()
{
    if (committed_passes_index_ >= committed_passes_.size())
    {
        return std::nullopt;
    }

    return committed_passes_.at(committed_passes_index_++);
}

void Strategy::commitPass(Pass pass)
{
    committed_passes_.push_back(pass);
}

Point Strategy::getNextBestReceivingPosition()
{
    if (receiving_positions_.empty())
    {
        std::vector<Point> existing_receiver_positions;
        std::transform(committed_passes_.begin(), committed_passes_.end(),
                       std::back_inserter(existing_receiver_positions),
                       [](const Pass& pass) { return pass.receiverPoint(); });

        unsigned int num_positions_to_generate = static_cast<unsigned int>(
            world_ptr_->friendlyTeam().numRobots() - existing_receiver_positions.size());

        receiving_positions_ = receiver_position_generator_.getBestReceivingPositions(
            *world_ptr_, num_positions_to_generate, existing_receiver_positions);
    }

    CHECK(receiving_positions_index_ < receiving_positions_.size())
        << "No more receiving positions to return";

    return receiving_positions_.at(receiving_positions_index_++);
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
    sampling_pass_generator_     = SamplingPassGenerator(ai_config_.passing_config());
    receiver_position_generator_ = ReceiverPositionGenerator<EighteenZoneId>(
        std::make_shared<EighteenZonePitchDivision>(Field::createSSLDivisionBField()),
        ai_config_.passing_config());
}

void Strategy::updateWorld(const WorldPtr& world_ptr)
{
    world_ptr_ = world_ptr;

    best_pass_.reset();

    committed_passes_.clear();
    committed_passes_index_ = 0;

    receiving_positions_.clear();
    receiving_positions_index_ = 0;

    robot_to_best_shot_.clear();
}
