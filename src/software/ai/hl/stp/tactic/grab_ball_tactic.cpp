#include "software/ai/hl/stp/tactic/grab_ball_tactic.h"

#include <limits>

#include "software/ai/evaluation/intercept.h"
#include "software/ai/hl/stp/action/intercept_ball_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/movespin_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/geom/rectangle.h"
#include "software/geom/util.h"
#include "software/util/logger/init.h"


GrabBallTactic::GrabBallTactic(const Field &field, const Ball &ball,
                               const Team &enemy_team, bool loop_forever)
    : Tactic(loop_forever), field(field), ball(ball), enemy_team(enemy_team)
{
}

std::string GrabBallTactic::getName() const
{
    return "Grab Ball Tactic";
}

void GrabBallTactic::updateParams(const Field &field, const Ball &ball,
                                  const Team &enemy_team)
{
    // Update the parameters stored by this Tactic
    this->field      = field;
    this->ball       = ball;
    this->enemy_team = enemy_team;
}

double GrabBallTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer robots closer to the intercept point if it exists, otherwise prefer robots
    // closer to the ball
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost    = 0.0;
    auto intercept = Evaluation::findBestInterceptForBall(ball, field, robot);
    if (intercept.has_value())
    {
        cost = dist(intercept->first, ball.position()) / world.field().totalXLength();
    }
    else
    {
        cost = dist(robot.position(), ball.position()) / world.field().totalXLength();
    }

    return std::clamp<double>(cost, 0, 1);
}

void GrabBallTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto intercept_action = std::make_shared<InterceptBallAction>(field, ball, true);
    auto movespin_action =
        std::make_shared<MoveSpinAction>(MoveSpinAction::ROBOT_CLOSE_TO_DEST_THRESHOLD);
    auto move_action =
        std::make_shared<MoveAction>(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD);

    auto intercept = Evaluation::findBestInterceptForBall(ball, field, *robot);

    do
    {
        double smallest_enemy_dist_to_ball = std::numeric_limits<double>::max();
        for (const auto &enemy : enemy_team.getAllRobotsExceptGoalie())
        {
            double enemy_dist = dist(enemy.position(), ball.position());
            smallest_enemy_dist_to_ball =
                std::min(enemy_dist, smallest_enemy_dist_to_ball);
        }

        if (smallest_enemy_dist_to_ball < BALL_DIST_FROM_ENEMY)
        {
            if (dist(robot->position(), ball.position()) < BALL_DIST_FROM_ENEMY)
            {
                movespin_action->updateControlParams(*robot, ball.position(),
                                                    STEAL_BALL_SPIN_SPEED, 0);
                yield(movespin_action);
            }
            else
            {
                move_action->updateControlParams(
                    *robot, ball.position(),
                    (ball.position() - robot->position()).orientation(), 0.0,
                    DribblerEnable::OFF, MoveType::NORMAL, AutokickType::NONE,
                    BallCollisionType::ALLOW);
                yield(move_action);
            }
        }
        else
        {
            intercept_action->updateWorldParams(field, ball);
            intercept_action->updateControlParams(*robot);
            yield(intercept_action);
        }
    } while (true);
}

void GrabBallTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
