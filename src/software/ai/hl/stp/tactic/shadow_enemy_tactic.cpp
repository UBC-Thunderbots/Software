#include "software/ai/hl/stp/tactic/shadow_enemy_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/robot.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"

ShadowEnemyTactic::ShadowEnemyTactic(const Field &field, const Team &friendly_team,
                                     const Team &enemy_team, bool ignore_goalie,
                                     const Ball &ball, const double ball_steal_speed,
                                     bool enemy_team_can_pass, bool loop_forever)
    : Tactic(loop_forever),
      field(field),
      friendly_team(friendly_team),
      enemy_team(enemy_team),
      ignore_goalie(ignore_goalie),
      ball(ball),
      ball_steal_speed(ball_steal_speed),
      enemy_team_can_pass(enemy_team_can_pass),
      shadow_distance(ROBOT_MAX_RADIUS_METERS * 3)
{
}

std::string ShadowEnemyTactic::getName() const
{
    return "Shadow Enemy Tactic";
}

void ShadowEnemyTactic::updateWorldParams(const Field &field, const Team &friendly_team,
                                          const Team &enemy_team, const Ball &ball)
{
    this->field         = field;
    this->friendly_team = friendly_team;
    this->enemy_team    = enemy_team;
    this->ball          = ball;
}

void ShadowEnemyTactic::updateControlParams(const EnemyThreat &enemy_threat,
                                            double shadow_distance)
{
    this->enemy_threat    = enemy_threat;
    this->shadow_distance = shadow_distance;
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
    double cost = (robot.position() - enemy_threat->robot.position()).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void ShadowEnemyTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto move_action = std::make_shared<MoveAction>(false, 0, Angle());
    auto stop_action = std::make_shared<StopAction>(true);

    do
    {
        if (!enemy_threat)
        {
            stop_action->updateControlParams(*robot, false);
            yield(stop_action);
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
                enemy_to_passer_vector.normalize(this->shadow_distance);
            move_action->updateControlParams(
                *robot, position_to_block_pass, enemy_to_passer_vector.orientation(), 0,
                DribblerEnable::OFF, MoveType::NORMAL, AutokickType::NONE,
                BallCollisionType::AVOID);
            yield(move_action);
        }
        else
        {
            std::vector<Robot> robots_to_ignore = {*robot};
            if (ignore_goalie && friendly_team.goalie())
            {
                robots_to_ignore.emplace_back(*friendly_team.goalie());
            }
            auto best_enemy_shot_opt =
                calcBestShotOnFriendlyGoal(field, friendly_team, enemy_team, enemy_robot,
                                           ROBOT_MAX_RADIUS_METERS, robots_to_ignore);
            Vector enemy_shot_vector = Vector(0, 0);
            if (best_enemy_shot_opt)
            {
                enemy_shot_vector =
                    best_enemy_shot_opt->getPointToShootAt() - enemy_robot.position();
            }
            else
            {
                enemy_shot_vector = field.friendlyGoalCenter() - enemy_robot.position();
            }

            Point position_to_block_shot =
                enemy_robot.position() +
                enemy_shot_vector.normalize(this->shadow_distance);

            // If the enemy robot already had the ball, try steal it and chip it away
            if (*robotHasPossession(ball.getPreviousStates(),
                                    enemy_robot.getPreviousStates()) &&
                ball.velocity().length() <= ball_steal_speed)
            {
                move_action->updateControlParams(
                    *robot, ball.position(),
                    (ball.position() - robot->position()).orientation(), 0,
                    DribblerEnable::ON, MoveType::NORMAL, AutokickType::AUTOCHIP,
                    BallCollisionType::AVOID);
                yield(move_action);
            }
            else
            {
                move_action->updateControlParams(
                    *robot, position_to_block_shot,
                    enemy_shot_vector.orientation() + Angle::half(), 0,
                    DribblerEnable::OFF, MoveType::NORMAL, AutokickType::NONE,
                    BallCollisionType::AVOID);
                yield(move_action);
            }
        }
    } while (!move_action->done());
}

void ShadowEnemyTactic::accept(MutableTacticVisitor &visitor)
{
    visitor.visit(*this);
}

Ball ShadowEnemyTactic::getBall() const
{
    return this->ball;
}

Field ShadowEnemyTactic::getField() const
{
    return this->field;
}

Team ShadowEnemyTactic::getFriendlyTeam() const
{
    return this->friendly_team;
}

Team ShadowEnemyTactic::getEnemyTeam() const
{
    return this->enemy_team;
}
