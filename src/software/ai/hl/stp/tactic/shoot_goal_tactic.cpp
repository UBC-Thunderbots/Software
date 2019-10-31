#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/intercept.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/geom/rectangle.h"
#include "software/util/parameter/dynamic_parameters.h"

ShootGoalTactic::ShootGoalTactic(const Field &field, const Team &friendly_team,
                                 const Team &enemy_team, const Ball &ball,
                                 Angle min_net_open_angle,
                                 std::optional<Point> chip_target, bool loop_forever)
    : Tactic(loop_forever, {RobotCapabilities::Capability::Kick}),
      field(field),
      friendly_team(friendly_team),
      enemy_team(enemy_team),
      ball(ball),
      min_net_open_angle(min_net_open_angle),
      chip_target(chip_target),
      has_shot_available(false)
{
    addWhitelistedAvoidArea(AvoidArea::BALL);
    addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
}

std::string ShootGoalTactic::getName() const
{
    return "Shoot Goal Tactic";
}

void ShootGoalTactic::updateWorldParams(const Field &field, const Team &friendly_team,
                                        const Team &enemy_team, const Ball &ball)
{
    this->field         = field;
    this->friendly_team = friendly_team;
    this->enemy_team    = enemy_team;
    this->ball          = ball;
}

void ShootGoalTactic::updateControlParams(std::optional<Point> chip_target)
{
    this->chip_target = chip_target;
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
               world.field().totalXLength();
    }
    else
    {
        // If we can't intercept the ball, just use the distance to the ball's current
        // position. We normalize with the total field length so that robots that are
        // within the field have a cost less than 1
        cost = (world.ball().position() - robot.position()).len() /
               world.field().totalXLength();
    }

    return std::clamp<double>(cost, 0, 1);
}

bool ShootGoalTactic::hasShotAvailable() const
{
    return has_shot_available;
}

bool ShootGoalTactic::isEnemyAboutToStealBall() const
{
    // Our rectangle class does not have the concept of rotation, so instead
    // we rotate all the robot positions about the origin so we can construct
    // a rectangle that is aligned with the axis
    Vector front_of_robot_dir =
        Vector(robot->orientation().cos(), robot->orientation().sin());

    auto steal_ball_rect_width = Util::DynamicParameters->getShootGoalTacticConfig()
                                     ->EnemyAboutToStealBallRectangleWidth()
                                     ->value();
    auto steal_ball_rect_length = Util::DynamicParameters->getShootGoalTacticConfig()
                                      ->EnemyAboutToStealBallRectangleExtensionLength()
                                      ->value();
    Rectangle baller_frontal_area = Rectangle(
        (robot->position() + front_of_robot_dir.perp().norm(steal_ball_rect_width / 2.0)),
        robot->position() + front_of_robot_dir.norm(steal_ball_rect_length) -
            front_of_robot_dir.perp().norm(ROBOT_MAX_RADIUS_METERS));

    for (const auto &enemy : enemy_team.getAllRobots())
    {
        if (baller_frontal_area.containsPoint(enemy.position()))
        {
            return true;
        }
    }

    return false;
}

void ShootGoalTactic::shootUntilShotBlocked(KickAction &kick_action,
                                            ChipAction &chip_action,
                                            IntentCoroutine::push_type &yield) const
{
    auto shot_target = Evaluation::calcBestShotOnEnemyGoal(field, friendly_team,
                                                           enemy_team, ball.position());
    while (shot_target)
    {
        if (!isEnemyAboutToStealBall())
        {
            yield(kick_action.updateStateAndGetNextIntent(
                *robot, ball, ball.position(), shot_target->getPointToShootAt(),
                BALL_MAX_SPEED_METERS_PER_SECOND - 0.5));
        }
        else
        {
            // If we are in the middle of committing to a shot but an enemy is about to
            // steal the ball we chip instead to just get over the enemy. We do not adjust
            // the point we are targeting since that may take more time to realign to, and
            // we need to be very quick so the enemy doesn't get the ball
            yield(chip_action.updateStateAndGetNextIntent(
                *robot, ball, ball.position(), shot_target->getPointToShootAt(),
                CHIP_DIST));
        }

        shot_target = Evaluation::calcBestShotOnEnemyGoal(field, friendly_team,
                                                          enemy_team, ball.position());
    }
}

void ShootGoalTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    KickAction kick_action = KickAction();
    ChipAction chip_action = ChipAction();
    MoveAction move_action =
        MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, Angle(), true);

    do
    {
        auto shot_target = Evaluation::calcBestShotOnEnemyGoal(
            field, friendly_team, enemy_team, ball.position());
        if (shot_target && shot_target->getOpenAngle() > min_net_open_angle)
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
            Point fallback_chip_target = chip_target ? *chip_target : field.enemyGoal();
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
                *robot, behind_ball, (-behind_ball_vector).orientation(), 0,
                DribblerEnable::OFF, MoveType::NORMAL, AutokickType::NONE));
        }
    } while (!(kick_action.done() || chip_action.done()));
}

void ShootGoalTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
