#pragma once

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class MovePrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new Move Primitive
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param dest The final destination of the movement
     * @param final_angle The final orientation the robot should have at the end
     * of the movement
     * @param final_speed The final speed the Robot should have when it reaches
     * its destination at the end of the movement
     */
    explicit MovePrimitive(unsigned int robot_id, const Point &dest,
                           const Angle &final_angle, double final_speed);

    /**
     * Creates a new Move Primitive from a Primitive message
     *
     * @param primtiive_msg The message from which to create the Move Primitive
     */
    explicit MovePrimitive(const thunderbots_msgs::Primitive &primtive_msg);

    /**
     * Gets the primitive name
     *
     * @return The name of the primitive as a string
     */
    std::string getPrimitiveName() const;

    /**
     * Gets the robot ID
     *
     * @return The robot ID as an unsigned integer
     */
    unsigned int getRobotId() const override;

    /**
     * gets the robots destination
     *
     * @return The robots destination as a Point(X,Y)
     */
    Point getDestination() const;

    /**
     * Gets the robots destination orientation
     *
     * @return The robots final orientation as an Angle
     */
    Angle getFinalAngle() const;

    /**
     * Gets the robots final speed in m/s
     *
     * @return The robots speed in m/s
     */
    double getFinalSpeed() const;

    /**
     * Gets the parameter array of the primitive
     *
     * @return The parameter array of the primitive as a vector<double>
     */
    std::vector<double> getParameterArray() const override;

    /**
     * gets the extra bit array of the primitive
     *
     * @return The extra bit array as a vector<bool>
     */
    std::vector<bool> getExtraBitArray() const override;

   private:
    unsigned int robot_id;
    Point dest;
    Angle final_angle;
    double final_speed;
};
