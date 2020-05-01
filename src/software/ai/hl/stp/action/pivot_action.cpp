#include "software/ai/hl/stp/action/pivot_action.h"

#include "shared/constants.h"
#include "software/ai/intent/move_intent.h"
#include "software/ai/intent/pivot_intent.h"
#include "software/geom/util.h"
#include "software/logger/logger.h"
#include "software/new_geom/angle.h"
#include "software/parameter/dynamic_parameters.h"

PivotAction::PivotAction() : Action(true) {}

void PivotAction::updateControlParams(const Robot& robot, Point pivot_point,
                                      Angle final_angle, Angle pivot_speed,
                                      DribblerEnable enable_dribbler)
{
    this->robot           = robot;
    this->pivot_point     = pivot_point;
    this->final_angle     = final_angle;
    this->pivot_speed     = pivot_speed;
    this->enable_dribbler = enable_dribbler;
}

void PivotAction::accept(MutableActionVisitor& visitor)
{
    visitor.visit(*this);
}

void PivotAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // If we're not in position to pivot, move to grab the ball
    if (!(robot->position()).isClose(pivot_point, 1.15))
    {
        yield(std::make_unique<MoveIntent>(
            robot->id(), pivot_point, (pivot_point - robot->position()).orientation(),
            0.0, 0, enable_dribbler ? DribblerEnable::ON : DribblerEnable::OFF,
            MoveType::NORMAL, AutokickType::NONE, BallCollisionType::AVOID));
        LOG(DEBUG) << "obtaining ball, moving!";
    }
    else
    {
        // if the robot is close enough to the final position, call it a day
        Angle threshold_angle = Angle::fromDegrees(5);

        if (robot->orientation() >= (final_angle - threshold_angle) &&
            robot->orientation() < (final_angle + threshold_angle))
        {
            LOG(DEBUG) << "Pivot angle reached threshold";
            return;
        }

        yield(std::make_unique<PivotIntent>(robot->id(), pivot_point, final_angle,
                                            pivot_speed, enable_dribbler, 0));
    }
}
