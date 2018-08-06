#pragma once

#include "ai/intent/intent.h"
#include "geom/angle.h"
#include "geom/point.h"

class MoveIntent : public Intent
{
   public:
    /**
     * Creates a new Move Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param dest The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     */
    // TODO: Add parameter override field/object/struct
    explicit MoveIntent(
        unsigned int robot_id, const Point &dest, const Angle &final_angle,
        double final_speed);

    std::string getIntentName() const override;

    unsigned int getRobotId() const override;

    /**
     * Returns the destination of this movement
     *
     * @return the destination of this movement
     */
    Point getDestination() const;

    /**
     * Returns the final orientation the robot should have at the
     * end of its movement
     *
     * @return the final orientation the robot should have at the end of its movement
     */
    Angle getFinalAngle() const;

    /**
     * Returns the final speed the robot should have when it arrives
     * at its destination
     *
     * @return the final speed the robot should have when it arrives
     * at its destination
     */
    double getFinalSpeed() const;

   private:
    unsigned int robot_id;
    Point dest;
    Angle final_angle;
    double final_speed;
};
