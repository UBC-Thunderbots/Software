#include "ai/hl/stp/action/pivot_action.h"

#include "ai/intent/move_intent.h"
#include "ai/intent/pivot_intent.h"
#include "geom/angle.h"
#include "geom/util.h"
#include "shared/constants.h"

PivotAction::PivotAction() : Action() {}

std::unique_ptr<Intent> PivotAction::updateStateAndGetNextIntent(const Robot& robot,
                                                                 Point pivot_point,
                                                                 Angle final_angle,
                                                                 Angle pivot_speed,
                                                                 bool enable_dribbler)
{
    // update the parameters stored by this action
    this->robot           = robot;
    this->pivot_point     = pivot_point;
    this->final_angle     = final_angle;
    this->pivot_speed     = pivot_speed;
    this->enable_dribbler = enable_dribbler;

    return getNextIntent();
}

void PivotAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    const double pivot_radius = BALL_MAX_RADIUS_METERS;
    do
    {
        // Compute collinear point (point_of_entry), radius with away,
        // between the robot and the pivot point. To move there before
        // attempting to pivot
        //
        //                             |---radius---|
        // (__) ---------------------- o -----------X
        // robot                 point_of_entry

        Vector unit_pivot_point_to_robot_pos =
            (robot->position() - this->pivot_point).norm();
        Point point_of_entry =
            this->pivot_point + pivot_radius * unit_pivot_point_to_robot_pos;

        // If we're not in position to pivot, move into position
        if ((robot->position() - point_of_entry).len() > ROBOT_MAX_RADIUS_METERS)
        {
            yield(std::make_unique<MoveIntent>(robot->id(), point_of_entry, Angle::zero(),
                                               0.0, 0));
        }
        else
        {
            yield(std::make_unique<PivotIntent>(robot->id(), pivot_point, final_angle,
                                                pivot_speed, enable_dribbler, 0));
        }
    } while (true);
}
