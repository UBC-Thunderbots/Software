#include "ai/hl/stp/action/pivot_action.h"

#include "ai/intent/move_intent.h"
#include "ai/intent/pivot_intent.h"
#include "geom/angle.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/logger/init.h"
#include "util/parameter/dynamic_parameters.h"

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
    // Compute final position, to know when the robot is done
    const double pivot_radius = BALL_MAX_RADIUS_METERS;

    do
    {
        // If we're not in position to pivot, move to grab the ball
        if (!(robot->position()).isClose(pivot_point, ROBOT_MAX_RADIUS_METERS*1.25))
        {
            yield(std::make_unique<MoveIntent>(robot->id(), pivot_point, (pivot_point-robot->position()).orientation(),
                                               0.0, 0, enable_dribbler));
            LOG(DEBUG)<<"obtaining ball, moving!";
        }
        else if (Util::DynamicParameters::PivotAction::experimental.value())
        {
            LOG(DEBUG)<<"EXPERIMENTAL ENABLED: ball has been obtainined, pivoting!";

            // if the robot is close enough to the final poision, call it a day
            Angle threshold_angle = Angle::ofDegrees(Util::DynamicParameters::PivotAction::finish_angle_threshold.value()/2);
            if(robot->orientation() >= (final_angle - threshold_angle) && robot->orientation() < (final_angle + threshold_angle)){
                break;
            }

            yield(std::make_unique<PivotIntent>(robot->id(), pivot_point, final_angle,
                                                pivot_speed, enable_dribbler, 0));
        }


    } while (true);
}
