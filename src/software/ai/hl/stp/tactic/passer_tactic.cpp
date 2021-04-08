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
    double cost = (robot.position() - world.ball().position()).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void PasserTactic::calculateNextAction(ActionCoroutine::push_type& yield)
{
    // Move to a position just behind the ball (in the direction of the pass)
    // until it's time to perform the pass
    auto move_action = std::make_shared<MoveAction>(
        true, MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, Angle());

    Angle kick_direction;

    auto kick_action = std::make_shared<KickAction>();
    do
    {
        // We want the robot to move to the starting position for the shot and also
        // rotate to the correct orientation to face the shot
        kick_action->updateControlParams(*robot_, ball.position(), pass.receiverPoint(),
                                         pass.speed());
        yield(kick_action);

        // We want to keep trying to kick until the ball is moving along the pass
        // vector with sufficient velocity
        kick_direction = pass.passerOrientation();

    } while (!ball.hasBallBeenKicked(kick_direction));
}

void PasserTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}
