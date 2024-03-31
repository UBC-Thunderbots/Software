#include "software/ai/evaluation/scoring/skills/feasibility_visitor.h"

#include "software/ai/evaluation/keep_away.h"

FeasibilityVisitor::FeasibilityVisitor(const Robot& robot,
                                       std::shared_ptr<Strategy> strategy,
                                       const World& world)
    : current_feasibility_(0), robot_(robot), strategy_(strategy), world_(world)
{
}

void FeasibilityVisitor::visit(const KeepAwaySkill& skill)
{
    if (world_.field().pointInFriendlyHalf(robot_.position()) ||
        contains(world_.field().enemyDefenseArea().expand(0.1), robot_.position()))
    {
        current_feasibility_ = 0;
        return;
    }

    if (!shouldKeepAway(robot_, world_.enemyTeam(),
                        strategy_->getAiConfig()
                            .attacker_tactic_config()
                            .enemy_about_to_steal_ball_radius()))
    {
        current_feasibility_ = 0;
        return;
    }

    current_feasibility_ = 1;
}

void FeasibilityVisitor::visit(const PassSkill& skill)
{
    PassWithRating best_pass = (*strategy_)->getBestUncommittedPass();
    current_feasibility_ = best_pass.rating;
}

void FeasibilityVisitor::visit(const ShootSkill& skill)
{
    static constexpr double IDEAL_SHOT_OPEN_ANGLE_DEGREES = 60.0;
    static constexpr double SIGMOID_WIDTH                 = 0.25;

    std::optional<Shot> best_shot = (*strategy_)->getBestShot(robot_);
    if (!best_shot)
    {
        current_feasibility_ = 0;
        return;
    }

    double shot_origin_feasibility = rectangleSigmoid(
        world_.field().enemyThird(), best_shot->getOrigin(), SIGMOID_WIDTH);

    double shot_angle_feasibility =
        normalizeValueToRange(best_shot->getOpenAngle().toDegrees(), 0.0,
                              IDEAL_SHOT_OPEN_ANGLE_DEGREES, 0.0, 1.0);

    current_feasibility_ = shot_origin_feasibility * shot_angle_feasibility;
}

double FeasibilityVisitor::getFeasibility(Skill& skill)
{
    skill.accept(*this);

    LOG_IF(WARNING, current_feasibility_ < 0 || current_feasibility_ > 1)
        << "Feasibility score outside of range [0, 1]";

    return current_feasibility_;
}
