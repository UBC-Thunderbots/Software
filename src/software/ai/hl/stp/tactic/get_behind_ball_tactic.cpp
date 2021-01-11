#include "software/ai/hl/stp/tactic/get_behind_ball_tactic.h"

#include <algorithm>

GetBehindBallTactic::GetBehindBallTactic(bool loop_forever)
    : Tactic(loop_forever, {RobotCapability::Move}),
      fsm(),
      controls({.ball_location              = Point(0, 0),
                .chick_direction            = Angle::zero(),
                .size_of_region_behind_ball = 4 * ROBOT_MAX_RADIUS_METERS})
{
}

void GetBehindBallTactic::updateWorldParams(const World &world) {}

void GetBehindBallTactic::updateControlParams(const Point &ball_location,
                                              Angle chick_direction)
{
    controls.ball_location   = ball_location;
    controls.chick_direction = chick_direction;
}

double GetBehindBallTactic::calculateRobotCost(const Robot &robot,
                                               const World &world) const
{
    // Prefer robots closer to the destination
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - controls.ball_location).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void GetBehindBallTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto move_action =
        std::make_shared<MoveAction>(false, MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD,
                                     MoveAction::ROBOT_CLOSE_TO_ORIENTATION_THRESHOLD);
    do
    {
        Vector behind_ball =
            Vector::createFromAngle(controls.chick_direction + Angle::half());
        Point point_behind_ball =
            controls.ball_location +
            behind_ball.normalize(controls.size_of_region_behind_ball * 3 / 4);
        move_action->updateControlParams(*robot_, point_behind_ball,
                                         controls.chick_direction, 0.0, DribblerMode::OFF,
                                         BallCollisionType::AVOID);
        yield(move_action);
    } while (!move_action->done());
}

bool GetBehindBallTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void GetBehindBallTactic::updateIntent(const TacticUpdate &tactic_update)
{
    GetBehindBallTacticFSM::Update event{.controls = controls, .common = tactic_update};
    fsm.process_event(event);
}

void GetBehindBallTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
