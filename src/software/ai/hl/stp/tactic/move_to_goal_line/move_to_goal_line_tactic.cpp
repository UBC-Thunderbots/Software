#include "software/ai/hl/stp/tactic/move_to_goal_line/move_to_goal_line_fsm.h"
#include "software/ai/hl/stp/tactic/move_to_goal_line/move_to_goal_line_tactic.h"

#include <algorithm>

MoveToGoalLineTactic::MoveToGoalLineTactic(){
}

void MoveToGoalLineTactic::updateWorldParams(const World &world) {}

void MoveToGoalLineTactic::updateControlParams(const Point &ball_location,
                                              Angle chick_direction)
{
    control_params.ball_location   = ball_location;
    control_params.chick_direction = chick_direction;
}

double MoveToGoalLineTactic::calculateRobotCost(const Robot &robot,
                                               const World &world) const
{
    // Prefer robots closer to the goal line
    // We normalize with the total field length so that robots that are within the
    // field have a cost less than 1
    double cost = (robot.position() - world.field().friendlyGoalCenter()).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void MoveToGoalLineTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    double size_of_region_behind_ball = 4 * ROBOT_MAX_RADIUS_METERS;
    auto move_action =
            std::make_shared<MoveAction>(false, MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD,
                                         MoveAction::ROBOT_CLOSE_TO_ORIENTATION_THRESHOLD);
    do
    {
        Vector behind_ball =
                Vector::createFromAngle(control_params.chick_direction + Angle::half());
        Point point_behind_ball =
                control_params.ball_location +
                behind_ball.normalize(size_of_region_behind_ball * 3 / 4);
        move_action->updateControlParams(*robot_, point_behind_ball,
                                         control_params.chick_direction, 0.0,
                                         DribblerMode::OFF, BallCollisionType::AVOID);
        yield(move_action);
    } while (!move_action->done());
}

bool MoveToGoalLineTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void MoveToGoalLineTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(GetBehindBallFSM::Update(control_params, tactic_update));
}

void MoveToGoalLineTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}