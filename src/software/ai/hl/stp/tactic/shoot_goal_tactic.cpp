#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/intercept.h"
#include "software/ai/hl/stp/action/intercept_ball_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/rectangle.h"

ShootGoalTactic::ShootGoalTactic(
    const Field &field, const Team &friendly_team, const Team &enemy_team,
    const Ball &ball, Angle min_net_open_angle, std::optional<Point> chip_target,
    bool loop_forever,
    std::shared_ptr<const ShootGoalTacticConfig> shoot_goal_tactic_config)
    : Tactic(loop_forever, {RobotCapability::Kick, RobotCapability::Move}),
      field(field),
      friendly_team(friendly_team),
      enemy_team(enemy_team),
      ball(ball),
      min_net_open_angle(min_net_open_angle),
      chip_target(chip_target),
      has_shot_available(false),
      shoot_goal_tactic_config(shoot_goal_tactic_config)
{
}

void ShootGoalTactic::updateWorldParams(const World &world)
{
    this->field         = world.field();
    this->friendly_team = world.friendlyTeam();
    this->enemy_team    = world.enemyTeam();
    this->ball          = world.ball();
}

void ShootGoalTactic::updateControlParams(std::optional<Point> chip_target)
{
    this->chip_target = chip_target;
}

double ShootGoalTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    auto ball_intercept_opt =
        findBestInterceptForBall(world.ball(), world.field(), robot);
    double cost = 0;
    if (ball_intercept_opt)
    {
        // If we can intercept the ball, use the distance to the intercept point.
        // We normalize with the total field length so that robots that are within the
        // field have a cost less than 1
        cost = (ball_intercept_opt->first - robot.position()).length() /
               world.field().totalXLength();
    }
    else
    {
        // If we can't intercept the ball, just use the distance to the ball's current
        // position. We normalize with the total field length so that robots that are
        // within the field have a cost less than 1
        cost = (world.ball().position() - robot.position()).length() /
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
        Vector(robot_->orientation().cos(), robot_->orientation().sin());

    auto steal_ball_rect_width =
        shoot_goal_tactic_config->getEnemyAboutToStealBallRectangleWidth()->value();
    auto steal_ball_rect_length =
        shoot_goal_tactic_config->getEnemyAboutToStealBallRectangleExtensionLength()
            ->value();
    Rectangle baller_frontal_area = Rectangle(
        (robot_->position() +
         front_of_robot_dir.perpendicular().normalize(steal_ball_rect_width / 2.0)),
        robot_->position() + front_of_robot_dir.normalize(steal_ball_rect_length) -
            front_of_robot_dir.perpendicular().normalize(ROBOT_MAX_RADIUS_METERS));

    for (const auto &enemy : enemy_team.getAllRobots())
    {
        if (contains(baller_frontal_area, enemy.position()))
        {
            return true;
        }
    }

    return false;
}

void ShootGoalTactic::shootUntilShotBlocked(std::shared_ptr<KickAction> kick_action,
                                            std::shared_ptr<ChipAction> chip_action,
                                            ActionCoroutine::push_type &yield) const
{
    std::optional<Shot> shot_target =
        calcBestShotOnGoal(field, friendly_team, enemy_team, ball.position(),
                           TeamType::ENEMY, {*this->getAssignedRobot()});

    while (shot_target && shot_target->getOpenAngle() > min_net_open_angle)
    {
        if (!isEnemyAboutToStealBall())
        {
            kick_action->updateControlParams(*robot_, ball.position(),
                                             shot_target->getPointToShootAt(),
                                             BALL_MAX_SPEED_METERS_PER_SECOND - 0.5);
            yield(kick_action);
        }
        else
        {
            // If we are in the middle of committing to a shot but an enemy is about to
            // steal the ball we chip instead to just get over the enemy. We do not adjust
            // the point we are targeting since that may take more time to realign to, and
            // we need to be very quick so the enemy doesn't get the ball
            chip_action->updateControlParams(*robot_, ball.position(),
                                             shot_target->getPointToShootAt());
            yield(chip_action);
        }
        shot_target =
            calcBestShotOnGoal(field, friendly_team, enemy_team, ball.position(),
                               TeamType::ENEMY, {*this->getAssignedRobot()});
    }
}

void ShootGoalTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto kick_action = std::make_shared<KickAction>();
    auto chip_action = std::make_shared<ChipAction>();
    auto move_action = std::make_shared<MoveAction>(
        true, MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, Angle());
    auto intercept_action = std::make_shared<InterceptBallAction>(field, ball);
    std::optional<Shot> shot_target;
    do
    {
        shot_target =
            calcBestShotOnGoal(field, friendly_team, enemy_team, ball.position(),
                               TeamType::ENEMY, {*this->getAssignedRobot()});

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
            Point fallback_chip_target =
                chip_target ? *chip_target : field.enemyGoalCenter();
            chip_action->updateControlParams(*robot_, ball.position(),
                                             fallback_chip_target);
            yield(chip_action);
        }
        else
        {
            intercept_action->updateControlParams(*robot_);
            yield(intercept_action);
        }
    } while (!(kick_action->done() || chip_action->done()));
}

void ShootGoalTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

Ball ShootGoalTactic::getBall() const
{
    return this->ball;
}

Field ShootGoalTactic::getField() const
{
    return this->field;
}

Team ShootGoalTactic::getFriendlyTeam() const
{
    return this->friendly_team;
}

Team ShootGoalTactic::getEnemyTeam() const
{
    return this->enemy_team;
}
