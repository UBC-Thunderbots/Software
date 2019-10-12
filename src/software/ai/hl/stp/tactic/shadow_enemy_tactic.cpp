#include "software/ai/hl/stp/tactic/shadow_enemy_tactic.h"

#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/evaluation/robot.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"

ShadowEnemyTactic::ShadowEnemyTactic(const Field &field, const Team &friendly_team,
                                     const Team &enemy_team, bool ignore_goalie,
                                     const Ball &ball, const double ball_steal_speed,
                                     bool loop_forever)
    : field(field),
      friendly_team(friendly_team),
      enemy_team(enemy_team),
      shadow_distance(ROBOT_MAX_RADIUS_METERS * 3),
      ignore_goalie(ignore_goalie),
      ball(ball),
      Tactic(loop_forever)
{
}

std::string ShadowEnemyTactic::getName() const
{
    return "Shadow Enemy Tactic";
}

void ShadowEnemyTactic::updateParams(const Evaluation::EnemyThreat &enemy_threat,
                                     const Field &field, const Team &friendly_team,
                                     const Team &enemy_team, double shadow_distance,
                                     bool enemy_team_can_pass, const Ball &ball)
{
    this->enemy_threat        = enemy_threat;
    this->field               = field;
    this->friendly_team       = friendly_team;
    this->enemy_team          = enemy_team;
    this->shadow_distance     = shadow_distance;
    this->enemy_team_can_pass = enemy_team_can_pass;
    this->ball                = ball;
    this->ball_steal_speed    = ball_steal_speed;
}

double ShadowEnemyTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    if (!enemy_threat)
    {
        return 0;
    }
    // Prefer robots closer to the enemy being shadowed
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - enemy_threat->robot.position()).len() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void ShadowEnemyTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    MoveAction move_action = MoveAction(0, Angle(), false);
    StopAction stop_action =
        StopAction(StopAction::ROBOT_STOPPED_SPEED_THRESHOLD_DEFAULT, true);
    do
    {
        if (!enemy_threat)
        {
            yield(stop_action.updateStateAndGetNextIntent(*robot, false));
        }

        Robot enemy_robot = enemy_threat->robot;
        // If we think the enemy team can pass, and if we have identified a robot that can
        // pass to the robot we are shadowing, we block the pass rather than block the
        // net. Otherwise, we just block the net
        if (enemy_team_can_pass && enemy_threat->passer)
        {
            Vector enemy_to_passer_vector =
                enemy_threat->passer->position() - enemy_robot.position();
            Point position_to_block_pass =
                enemy_robot.position() +
                enemy_to_passer_vector.norm(this->shadow_distance);
            yield(move_action.updateStateAndGetNextIntent(
                *robot, position_to_block_pass, enemy_to_passer_vector.orientation(), 0));
        }
        else
        {
            std::vector<Robot> robots_to_ignore = {*robot};
            if (ignore_goalie && friendly_team.goalie())
            {
                robots_to_ignore.emplace_back(*friendly_team.goalie());
            }
            auto best_enemy_shot_opt = Evaluation::calcBestShotOnFriendlyGoal(
                field, friendly_team, enemy_team, enemy_robot, ROBOT_MAX_RADIUS_METERS,
                robots_to_ignore);
            Vector enemy_shot_vector = Vector(0, 0);
            if (best_enemy_shot_opt)
            {
                enemy_shot_vector =
                    best_enemy_shot_opt->getPointToShootAt() - enemy_robot.position();
            }
            else
            {
                enemy_shot_vector = field.friendlyGoal() - enemy_robot.position();
            }

            Point position_to_block_shot =
                enemy_robot.position() + enemy_shot_vector.norm(this->shadow_distance);

            // If the enemy robot already had the ball, try steal it and chip it away
            if (Evaluation::robotHasPossession(ball, enemy_robot) &&
                ball.velocity().len() < ball_steal_speed)
            {
                yield(move_action.updateStateAndGetNextIntent(
                    *robot, ball.position(),
                    (ball.position() - robot->position()).orientation(), 0, true, false,
                    AutokickType::AUTOCHIP));
            }
            else
            {
                yield(move_action.updateStateAndGetNextIntent(
                    *robot, position_to_block_shot,
                    enemy_shot_vector.orientation() + Angle::half(), 0));
            }
        }
    } while (!move_action.done());
}

void ShadowEnemyTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
