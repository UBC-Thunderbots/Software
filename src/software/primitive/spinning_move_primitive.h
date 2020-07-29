#pragma once

#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/point.h"
#include "software/primitive/primitive.h"

class SpinningMovePrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new SpinningMove Primitive
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param dest The final destination of the movement
     * @param angular_vel The angular velocity of the robot
     * of the movement
     * @param final_speed The speed at final destination
     */
    explicit SpinningMovePrimitive(unsigned int robot_id, const Point &dest,
                                   const AngularVelocity &angular_vel,
                                   double final_speed);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    /**
     * Gets the robot's destination
     *
     * @return The robot's destination as a Point(X,Y)
     */
    Point getDestination() const;

    /**
     * Gets the robot's angular velocity in rad/s
     *
     * @return The robot's angular velocity in rad/s
     */
    AngularVelocity getAngularVelocity() const;

    /**
     * Gets the robot's final speed in m/s
     *
     * @return The robots speed in m/s
     */
    double getFinalSpeed() const;

    void accept(PrimitiveVisitor &visitor) const override;

    /**
     * Compares SpinningMovePrimitives for equality. SpinningMovePrimitives are considered
     * equal if all their member variables are equal.
     *
     * @param other the SpinningMovePrimitive to compare with for equality
     *
     * @return true if the SpinningMovePrimitives are equal and false otherwise
     */
    bool operator==(const SpinningMovePrimitive &other) const;

    /**
     * Compares SpinningMovePrimitives for inequality.
     *
     * @param other the SpinningMovePrimitive to compare with for inequality
     *
     * @return true if the SpinningMovePrimitives are not equal and false otherwise
     */
    bool operator!=(const SpinningMovePrimitive &other) const;

   private:
    unsigned int robot_id;
    Point dest;
    AngularVelocity angular_vel;
    double final_speed;
};
