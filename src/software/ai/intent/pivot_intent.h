#pragma once

#include "software/ai/intent/intent.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/primitive/pivot_primitive.h"

class PivotIntent : public PivotPrimitive, public Intent
{
   public:
    static const std::string INTENT_NAME;
    /**
     * Creates a new Pivot Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param dest The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param pivot_speed The angular speed the robot should have while it pivots
     * @param enable_dribbler Whether or not the dribbler should be spinning to hold the
     * ball
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     */
    explicit PivotIntent(unsigned int robot_id, const Point& pivot_point,
                         const Angle& final_angle, const Angle& pivot_speed,
                         bool enable_dribbler, unsigned int priority);

    std::string getIntentName(void) const override;

    void accept(IntentVisitor& visitor) const override;

    /**
     * Compares PivotIntents for equality. PivotIntents are considered equal if all
     * their member variables are equal.
     *
     * @param other the PivotIntents to compare with for equality
     * @return true if the PivotIntents are equal and false otherwise
     */
    bool operator==(const PivotIntent& other) const;

    /**
     * Compares PivotIntents for inequality.
     *
     * @param other the PivotIntent to compare with for inequality
     * @return true if the PivotIntents are not equal and false otherwise
     */
    bool operator!=(const PivotIntent& other) const;
};
