/**
 * Implementation of the ReceiverTactic
 */
#include "ai/hl/stp/tactic/receiver_tactic.h"

#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/logger/init.h"

using namespace AI::Passing;
using namespace Evaluation;

ReceiverTactic::ReceiverTactic(const Field& field, const Team& friendly_team,
                               const Team& enemy_team, const AI::Passing::Pass pass,
                               const Ball& ball, bool loop_forever)
    : field(field),
      pass(std::move(pass)),
      ball(ball),
      friendly_team(friendly_team),
      enemy_team(enemy_team),
      Tactic(loop_forever)
{
}

std::string ReceiverTactic::getName() const
{
    return "Receiver Tactic";
}

void ReceiverTactic::updateParams(const Team& updated_friendly_team,
                                  const Team& updated_enemy_team,
                                  const AI::Passing::Pass& updated_pass,
                                  const Ball& updated_ball)
{
    this->friendly_team = updated_friendly_team;
    this->enemy_team    = updated_enemy_team;
    this->ball          = updated_ball;
    this->pass          = updated_pass;
}

double ReceiverTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the pass receive position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost =
        (robot.position() - pass.receiverPoint()).len() / world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

std::unique_ptr<Intent> ReceiverTactic::calculateNextIntent(
    intent_coroutine::push_type& yield)
{
    MoveAction move_action = MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, true);
    while (ball.lastUpdateTimestamp() < pass.startTime())
    {
        // We want the robot to move to the receiving position for the shot and also
        // rotate to the correct orientation to face where the pass is coming from
        yield(move_action.updateStateAndGetNextIntent(*robot, pass.receiverPoint(),
                                                      pass.receiverOrientation(), 0));
        std::cout << "MOVING TO POSITION OR WAITING - PASS NOT STARTED" << std::endl;
    }

    // Check if we can shoot on the enemy goal from the receiver position
    std::optional<std::pair<Point, Angle>> best_shot_opt =
            calcBestShotOnEnemyGoal(field, friendly_team, enemy_team, );

    // Vector from the ball to the robot
    Vector robot_to_ball = ball.position() - robot->position();

    // If we have a shot with a sufficiently large enough opening, and the deflection
    // angle that is reasonable, we should one-touch kick the ball towards the enemy net
    // TODO: This conditional is _horrendous_, clean it up
    if (best_shot_opt &&
        // best_shot_opt->second.toDegrees() > MIN_SHOT_OPEN_ANGLE_DEGREES &&
        (robot_to_ball.orientation() -
         (best_shot_opt->first - robot->position()).orientation())
                .angleMod()
                .abs()
                .toDegrees() < MAX_DEFLECTION_FOR_ONE_TOUCH_SHOT_DEGREES)
    {
        auto [best_shot_target, _] = *best_shot_opt;

        // The angle between the ball velocity and a vector from the ball to the robot
        Vector ball_velocity = ball.velocity();
        robot_to_ball        = robot->position() - ball.position();
        Angle ball_robot_angle =
            (ball_velocity.orientation() - robot_to_ball.orientation()).angleMod();

        // TODO: Fix termination condition here
        //        while(ball_robot_angle.toDegrees() < 90 || ball.velocity().len() < 0.5)
        //        {
        while (true)
        {
            // Figure out the closest point on the balls trajectory to the robot
            Point closest_ball_pos = closestPointOnLine(
                robot->position(), ball.position(),
                ball.estimatePositionAtFutureTime(Duration::fromSeconds(0.1)));
            Ray shot(closest_ball_pos, best_shot_target - closest_ball_pos);

            // Determine the best position to be in
            Angle ideal_orientation = getOneTimeShotDirection(shot, ball);
            Vector ideal_orientation_vec =
                Vector::createFromAngle(ideal_orientation);  // Vector(ideal_orientation.cos(),
                                                             // ideal_orientation.sin());

            // The best position is determined such that the robot stays in the ideal
            // orientation, but moves forwards/backwards so that the ball hits the kicker,
            // rather then the center of the robot
            Point ideal_position = closest_ball_pos - ideal_orientation_vec.norm(
                                                          DIST_TO_FRONT_OF_ROBOT_METERS +
                                                          BALL_MAX_RADIUS_METERS);

            std::cout << "In ONe TOUCH" << std::endl;

            yield(move_action.updateStateAndGetNextIntent(
                *robot, ideal_position, ideal_orientation, 0, false, true));

            // Calculations to check for termination conditions
            ball_velocity = ball.velocity();
            robot_to_ball = ball.position() - robot->position();
            ball_robot_angle =
                (ball_velocity.orientation() - robot_to_ball.orientation()).angleMod();
        }
        std::cout << "DONE" << std::endl;
    }
    // If we can't shoot on the enemy goal, just try to receive the pass as cleanly as
    // possible
    else
    {
        while ((ball.position() - robot->position()).len() >
               DIST_TO_FRONT_OF_ROBOT_METERS + 0.03)
        {
            Point ball_receive_pos = closestPointOnLine(
                robot->position(), ball.position(),
                ball.estimatePositionAtFutureTime(Duration::fromSeconds(0.1)));
            Angle ball_receive_orientation =
                (ball.position() - robot->position()).orientation();

            std::cout << "TRYING TO CATCH WITH DRIBBLE" << std::endl;

            yield(move_action.updateStateAndGetNextIntent(
                *robot, ball_receive_pos, ball_receive_orientation, 0, true, false));
        }
        std::cout << "DONE" << std::endl;
    }
}

Angle ReceiverTactic::getOneTimeShotDirection(const Ray& shot, const Ball& ball)
{
    Point shot_vector = shot.getDirection();
    Angle shot_dir    = shot.getDirection().orientation();
    Point bot_vector  = shot_vector.norm();

    // TODO: magic numbers here??
    Point ball_vel       = ball.velocity();
    Point lateral_vel    = ball_vel - (ball_vel.dot(-bot_vector)) * (-bot_vector);
    double lateral_speed = 0.3 * lateral_vel.len();
    double kick_speed    = MAX_BALL_SPEED_METERS_PER_SECOND - 1;
    Angle shot_offset    = Angle::asin(lateral_speed / kick_speed);

    // check which direction the ball is going in so we can decide which direction to
    // apply the offset in
    if (lateral_vel.dot(shot_vector.rotate(Angle::quarter())) > 0)
    {
        // need to go clockwise
        shot_offset = -shot_offset;
    }
    return shot_dir + shot_offset;
}
