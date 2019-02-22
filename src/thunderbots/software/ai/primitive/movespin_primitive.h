#pragma once

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class MoveSpinPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new MoveSpin Primitive
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param dest The final destination of the movement
     * @param angular_vel The angular velocity of the robot
     * of the movement
     */
    explicit MoveSpinPrimitive(unsigned int robot_id, const Point &dest,
                               const AngularVelocity &angular_vel);

    /**
     * Creates a new MoveSpin Primitive from a Primitive message
     *
     * @param primitive_msg The message from which to create the Move Primitive
     */
    explicit MoveSpinPrimitive(const thunderbots_msgs::Primitive &primitive_msg);

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
     * Returns the generic vector of parameters for this Primitive
     *
     * @return A vector of the form {dest.x(), dest.y(), angular_vel.toRadians()}
     */
    std::vector<double> getParameters() const override;

    /**
     * This primitive has no extra bits
     *
     * @return an empty vector
     */
    std::vector<bool> getExtraBits() const override;

    void accept(PrimitiveVisitor &visitor) const override;

    /**
     * Compares MoveSpinPrimitives for equality. MoveSpinPrimitives are considered equal
     * if all their member variables are equal.
     *
     * @param other the MoveSpinPrimitive to compare with for equality
     * @return true if the MoveSpinPrimitives are equal and false otherwise
     */
    bool operator==(const MoveSpinPrimitive &other) const;

    /**
     * Compares MoveSpinPrimitives for inequality.
     *
     * @param other the MoveSpinPrimitive to compare with for inequality
     * @return true if the MoveSpinPrimitives are not equal and false otherwise
     */
    bool operator!=(const MoveSpinPrimitive &other) const;

   private:
    unsigned int robot_id;
    Point dest;
    AngularVelocity angular_vel;
};
