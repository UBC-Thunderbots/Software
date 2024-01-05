#include "software/ai/hl/stp/strategy/strategy.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.hpp"

Strategy::Strategy(const TbotsProto::AiConfig& ai_config)
{
    updateAiConfig(ai_config);
}

Pose Strategy::getBestDribblePose(const Robot& robot, const World& world)
{
    if (robot_to_best_dribble_location_.contains(robot.id()))
    {
        return robot_to_best_dribble_location_.at(robot.id());
    }

    // TODO(#3082): temporary logic, find best dribble_position
    Vector robot_to_goal = world.field().enemyGoalCenter() - robot.position();
    Point dribble_position = robot.position() + robot_to_goal.normalize(0.8);

    // cache the dribble position
    robot_to_best_dribble_location_[robot.id()] = Pose(dribble_position, robot_to_goal.orientation());

    return robot_to_best_dribble_location_.at(robot.id());
}

Pass Strategy::getBestPass(const Robot& robot, const World& world)
{
    if (robot_to_best_pass_.contains(robot.id()))
    {
        return robot_to_best_pass_.at(robot.id());
    }

    // calculate best pass
    pass_generator_->generatePassEvaluation(world);

    return Pass(Point(), Point(), 1);
}

std::optional<Shot> Strategy::getBestShot(const Robot& robot, const World& world)
{
    if (robot_to_best_shot_.contains(robot.id()))
    {
        return robot_to_best_shot_.at(robot.id());
    }

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
    robot_to_best_pass_             = {};
    robot_to_best_shot_             = {};
}

void Strategy::updateAiConfig(const TbotsProto::AiConfig& ai_config)
{
    ai_config_ = ai_config;
    
    {
        const std::lock_guard<std::mutex> lock(pass_generator_lock_)

        pass_generator_ = std::make_unique<PassGenerator<EighteenZoneId>>(
            std::make_shared<const EighteenZonePitchDivision>(Field::createSSLDivisionBField()),
            ai_config.passing_config());
    }

    reset();
}

void Strategy::updateWorld(const World& world)
{
    const std::lock_guard<std::mutex> lock(world_lock_);
    current_world_ = std::make_optional<std::reference_wrapper<const World>>(world);
}
