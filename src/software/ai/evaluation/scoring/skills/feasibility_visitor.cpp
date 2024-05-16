#include "software/ai/evaluation/scoring/skills/feasibility_visitor.h"

#include "software/ai/evaluation/keep_away.h"
#include "software/geom/algorithms/contains.h"

FeasibilityVisitor::FeasibilityVisitor(const Robot& robot,
                                       std::shared_ptr<Strategy> strategy,
                                       const World& world)
    : current_feasibility_(0), robot_(robot), strategy_(strategy), world_(world)
{
}

void FeasibilityVisitor::visit(const KeepAwaySkill& skill)
{
    if (!shouldKeepAway(robot_, world_.enemyTeam(),
                        strategy_->getAiConfig()
                            .attacker_tactic_config()
                            .enemy_about_to_steal_ball_radius()))
    {
        current_feasibility_ = 0;
        return;
    }

    current_feasibility_ = 0.7;
}

void FeasibilityVisitor::visit(const KickPassSkill& skill)
{
    PassWithRating best_pass = strategy_->getBestPass();
    
    current_feasibility_ = best_pass.rating * 0.9;

    Segment passing_lane(best_pass.pass.passerPoint(), best_pass.pass.receiverPoint());
    if (std::all_of(world_.enemyTeam().getAllRobots().begin(), 
                    world_.enemyTeam().getAllRobots().end(), 
                    [&](auto enemy) { return distance(enemy.position(), passing_lane) >= 0.15; }))
    {
        current_feasibility_ += 0.05;
    }
}

void FeasibilityVisitor::visit(const ChipPassSkill& skill)
{
    PassWithRating best_pass = strategy_->getBestPass();
    
    current_feasibility_ = best_pass.rating * 0.9;

    Segment passing_lane(best_pass.pass.passerPoint(), best_pass.pass.receiverPoint());
    if (std::any_of(world_.enemyTeam().getAllRobots().begin(), 
                    world_.enemyTeam().getAllRobots().end(), 
                    [&](auto enemy) { return distance(enemy.position(), passing_lane) < 0.15; }))
    {
        current_feasibility_ += 0.05;
    }
}

void FeasibilityVisitor::visit(const ShootSkill& skill)
{
    static constexpr double IDEAL_SHOT_OPEN_ANGLE_DEGREES = 40.0;
    static constexpr double SIGMOID_WIDTH                 = 0.25;

    std::optional<Shot> best_shot = strategy_->getBestShot(robot_);
    if (!best_shot)
    {
        current_feasibility_ = 0;
        return;
    }

    double shot_origin_feasibility = rectangleSigmoid(
        world_.field().enemyHalf(), best_shot->getOrigin(), SIGMOID_WIDTH);

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
