#include "software/ai/hl/stp/tactic/passer_tactic.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/action/intercept_ball_action.h"
#include "software/ai/hl/stp/action/kick_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/logger/logger.h"
#include "software/world/ball.h"

PasserTactic::PasserTactic(Pass pass, const Ball& ball, const Field& field,
                           bool loop_forever)
    : Tactic(loop_forever, {RobotCapability::Kick, RobotCapability::Move}),
      pass(std::move(pass)),
      ball(ball),
      field(field)
{
}

void PasserTactic::updateWorldParams(const World& world)
{
    this->ball  = world.ball();
    this->field = world.field();
}

void PasserTactic::updateControlParams(const Pass& updated_pass)
{
    this->pass = updated_pass;
}

double PasserTactic::calculateRobotCost(const Robot& robot, const World& world) const
{
    // Prefer robots closer to the pass start position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost =
        (robot.position() - world.ball().position()).length() / world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void PasserTactic::calculateNextAction(ActionCoroutine::push_type& yield)
{
    // If the ball is moving, we are likely already in a live game scenario and
    // so we need to collect the ball before we can pass. If the ball is not moving,
    // we are likely in a set play and so we don't need to initially collect the ball
    if (ball.velocity().length() > INTERCEPT_BALL_SPEED_THRESHOLD)
    {
        auto intercept_action = std::make_shared<InterceptBallAction>(field, ball);
        do
        {
            intercept_action->updateControlParams(*robot_);
            yield(intercept_action);
        } while (!intercept_action->done());
    }

    // Move to a position just behind the ball (in the direction of the pass)
    // until it's time to perform the pass
    auto move_action = std::make_shared<MoveAction>(
        true, MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, Angle());

    // TODO (ticket here) we shouldn't wait to take the pass here. The play should
    // align itself up to the ball and wait while looking at enemy shadowers, and
    // then pivot/pass when necessary.
    //
    // For now we allow for 2 second setup time, but this tactic shouldn't care.
    auto setup_time = ball.timestamp() + Duration::fromSeconds(2.0);
    while (ball.timestamp() < setup_time)
    {
        // The passer should be facing the receiver
        auto passer_orientation =
            pass.receiverOrientation(ball.position()) + Angle::fromDegrees(180);

        // We want to wait just behind where the pass is supposed to start, so that the
        // ball is *almost* touching the kicker
        Vector ball_offset =
            Vector::createFromAngle(passer_orientation)
                .normalize(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS * 2);
        Point wait_position = ball.position() - ball_offset;

        move_action->updateControlParams(*robot_, wait_position, passer_orientation,
                                         0, DribblerMode::OFF, BallCollisionType::ALLOW);
        yield(move_action);
    }

    Angle kick_direction;

    auto kick_action = std::make_shared<KickAction>();
    do
    {
        // We want the robot to move to the starting position for the shot and also
        // rotate to the clorrect orientation to face the shot
        kick_action->updateControlParams(*robot_, ball.position(), pass.receiverPoint(),
                                         pass.speed());
        yield(kick_action);

        // We want to keep trying to kick until the ball is moving along the pass
        // vector with sufficient velocity
        kick_direction = (pass.receiverPoint() - ball.position()).orientation();

    } while (!ball.hasBallBeenKicked(kick_direction));
}

void PasserTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

Ball PasserTactic::getBall() const
{
    return this->ball;
}
