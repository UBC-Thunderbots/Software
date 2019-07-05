#pragma once

#include "ai/hl/stp/action/action.h"
#include "geom/angle.h"
#include "geom/point.h"

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
     * Returns the next Intent this PivotAction wants to run, given the parameters.
     *
     * @param robot the robot that should perform the pivot
     * @param pivot_point the point around which the robot pivots
     * @param final_angle the absolute, not relative, final angle
     * @param pivot_speed angular speed to
     *
     * @return A unique pointer to the Intent the PivotAction wants to run. If the
     * PivotAction is done, returns an empty/null pointer
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(const Robot& robot,
                                                        Point pivot_point,
                                                        Angle final_angle,
                                                        Angle pivot_speed,
                                                        bool enable_dribbler);

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    Point pivot_point;
    Angle final_angle;
    Angle pivot_speed;
    bool enable_dribbler;
};
