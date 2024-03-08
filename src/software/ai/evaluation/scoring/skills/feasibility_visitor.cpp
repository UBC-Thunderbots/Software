#include "software/ai/evaluation/scoring/skills/feasibility_visitor.h"

#include "software/ai/evaluation/keep_away.h"

FeasibilityVisitor::FeasibilityVisitor(const Robot& robot, std::shared_ptr<Strategy> strategy, const World& world)
    : current_feasibility_(0),
      robot_(robot),
      strategy_(strategy),
      world_(world)
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
                strategy_->getAiConfig().attacker_tactic_config().enemy_about_to_steal_ball_radius()))
    {
        current_feasibility_ = 0;
        return;
    }

    current_feasibility_ = 1;
}

void FeasibilityVisitor::visit(const PassSkill& skill)
{
    PassWithRating best_pass = (*strategy_)->getBestPass();

    double min_score = (*strategy_)->getAiConfig().shoot_or_pass_play_config().abs_min_pass_score();

    if (best_pass.rating > min_score)
    {
        // TODO(arun): should we normalize this?
        current_feasibility_ = best_pass.rating;
        return;
    }

    current_feasibility_ = 0;
    return;
}

void FeasibilityVisitor::visit(const ShootSkill& skill)
{
    std::optional<Shot> best_shot = (*strategy_)->getBestShot(robot_);
    if (!best_shot)
    {
        current_feasibility_ = 0;
        return;
    }

    double shot_angle_viability =
        normalizeValueToRange(best_shot->getOpenAngle().toDegrees(), 0.0, 60.0, 0.0, 1.0);
    current_feasibility_ = shot_angle_viability;
}

double FeasibilityVisitor::getFeasibility(const Skill& skill)
{
    skill.accept(*this);

    return current_feasibility_;
}
