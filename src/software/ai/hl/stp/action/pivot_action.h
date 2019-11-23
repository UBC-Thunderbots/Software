#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/ai/hl/stp/action/action_visitor.h"
#include "software/ai/intent/move_intent.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"

/**
 * The PivotAction makes the robot pivot from the given pivot point to the given
 * angle with the given radius
 */
class PivotAction : public Action
{
   public:
    /**
     * Creates a new PivotAction
     */
    explicit PivotAction();

    /**
     * Updates the params that cannot be derived from the world for this action
     *
     * @param robot the robot that should perform the pivot
     * @param pivot_point the point around which the robot pivots
     * @param final_angle the absolute, not relative, final angle
     * @param pivot_speed angular speed to
     */
    void updateControlParams(const Robot& robot, Point pivot_point, Angle final_angle,
                             Angle pivot_speed, DribblerEnable enable_dribbler);

    void accept(ActionVisitor& visitor) const override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    Point pivot_point;
    Angle final_angle;
    Angle pivot_speed;
    DribblerEnable enable_dribbler;
};
