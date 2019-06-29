#include "ai/hl/stp/tactic/shoot_goal_tactic.h"

#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "ai/hl/stp/evaluation/intercept.h"

ShootGoalTactic::ShootGoalTactic(const Field &field, const Team &friendly_team,
                                 const Team &enemy_team, const Ball &ball,
                                 double min_percent_net_open,
                                 std::optional<Point> chip_target, bool loop_forever)
    : field(field),
      friendly_team(friendly_team),
      enemy_team(enemy_team),
      ball(ball),
      min_percent_net_open(min_percent_net_open),
      chip_target(chip_target),
      has_shot_available(false),
      Tactic(loop_forever, {RobotCapabilityFlags::Kick})
{
}

std::string ShootGoalTactic::getName() const
{
    return "Shoot Goal Tactic";
}

void ShootGoalTactic::updateParams(const Field &field, const Team &friendly_team,
                                   const Team &enemy_team, const Ball &ball)
{
    this->field         = field;
    this->friendly_team = friendly_team;
    this->enemy_team    = enemy_team;
    this->ball          = ball;
}

double ShootGoalTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    auto ball_intercept_opt =
        Evaluation::findBestInterceptForBall(world.ball(), world.field(), robot);
    double cost = 0;
    if (ball_intercept_opt)
    {
        // If we can intercept the ball, use the distance to the intercept point.
        // We normalize with the total field length so that robots that are within the
        // field have a cost less than 1
        cost = (ball_intercept_opt->first - robot.position()).len() /
               world.field().totalLength();
    }
    else
    {
        // If we can't intercept the ball, just use the distance to the ball's current
        // position. We normalize with the total field length so that robots that are
        // within the field have a cost less than 1
        cost = (world.ball().position() - robot.position()).len() /
               world.field().totalLength();
    }

    return std::clamp<double>(cost, 0, 1);
}

bool ShootGoalTactic::hasShotAvailable() const
{
    return has_shot_available;
}

bool ShootGoalTactic::isEnemyAboutToStealBall() const
{
    // TODO: replace with get all robots except goalie?
    for (const auto &enemy : enemy_team.getAllRobots())
    {
        if ((enemy.position() - ball.position()).len() < ENEMY_DANGER_DIST)
        {
            return true;
        }
    }
    return false;
}

std::optional<std::pair<Point, double>> ShootGoalTactic::getShotData() const
{
    auto best_shot_opt = Evaluation::calcBestShotOnEnemyGoal(field, friendly_team,
                                                             enemy_team, ball.position());
    if (best_shot_opt)
    {
        double net_percent_open = Evaluation::calcShotOpenEnemyNetPercentage(
            field, ball.position(), *best_shot_opt);
        return std::make_pair(best_shot_opt->first, net_percent_open);
    }

    return std::nullopt;
}

void ShootGoalTactic::shootUntilShotBlocked(KickAction &kick_action,
                                            ChipAction &chip_action,
                                            IntentCoroutine::push_type &yield) const
{
    auto shot_target = getShotData();
    while (shot_target)
    {
        if (!isEnemyAboutToStealBall())
        {
            yield(kick_action.updateStateAndGetNextIntent(
                *robot, ball, ball.position(), shot_target->first,
                BALL_MAX_SPEED_METERS_PER_SECOND - 0.5));
        }
        else
        {
            // If we are in the middle of committing to a shot but an enemy is about to
            // steal the ball we chip instead to just get over the enemy. We do not adjust
            // the point we are targeting since that may take more time to realign to, and
            // we need to be very quick so the enemy doesn't get the ball
            yield(chip_action.updateStateAndGetNextIntent(*robot, ball, ball.position(),
                                                          shot_target->first, CHIP_DIST));
        }

        shot_target = getShotData();
    }
}

void ShootGoalTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    KickAction kick_action = KickAction();
    ChipAction chip_action = ChipAction();
    MoveAction move_action = MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, true);

    Point fallback_chip_target = chip_target ? *chip_target : field.enemyGoal();
    do
    {
        auto shot_target = getShotData();
        if (shot_target && shot_target->second > min_percent_net_open)
        {
            // Once we have determined we can take a shot, continue to try shoot until the
            // shot is entirely blocked
            has_shot_available = true;
            shootUntilShotBlocked(kick_action, chip_action, yield);
            has_shot_available = false;
        }
        else if (isEnemyAboutToStealBall())
        {
            // If an enemy is about to steal the ball from us, we try chip over them to
            // try recover the ball after, which is better than being stripped of the ball
            // and directly losing possession that way
            yield(chip_action.updateStateAndGetNextIntent(
                *robot, ball, ball.position(), fallback_chip_target, CHIP_DIST));
        }
        else
        {
            Vector behind_ball_vector = (ball.position() - field.enemyGoal());
            // A point behind the ball that leaves 5cm between the ball and kicker of the
            // robot
            Point behind_ball =
                ball.position() +
                behind_ball_vector.norm(BALL_MAX_RADIUS_METERS +
                                        DIST_TO_FRONT_OF_ROBOT_METERS + TRACK_BALL_DIST);

            // The default behaviour is to move behind the ball and face the net
            yield(move_action.updateStateAndGetNextIntent(
                *robot, behind_ball, (-behind_ball_vector).orientation(), 0));
        }
    } while (!(kick_action.done() || chip_action.done()));
}
