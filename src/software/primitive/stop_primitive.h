#pragma once

#include "software/primitive/primitive.h"

class StopPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;

    /**
     * Creates a new Stop Primitive
     *
     * Stops the robot with the option to coast to a stop rather than stop immediately
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param coast to coast to a stop or not
     */
    explicit StopPrimitive(unsigned int robot_id, bool coast);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    /**
     * Gets whether the robot should coast or not
     *
     * @return whether the robot should coast to a stop
     */
    bool robotShouldCoast() const;

    void accept(PrimitiveVisitor& visitor) const override;

    /**
     * Compares StopPrimitives for equality. StopPrimitives are considered equal if all
     * their member variables are equal.
     *
     * @param other the StopPrimitive to compare with for equality
     * @return true if the StopPrimitives are equal and false otherwise
     */
    bool operator==(const StopPrimitive& other) const;

    /**
     * Compares StopPrimitives for inequality.
     *
     * @param other the StopPrimitive to compare with for inequality
     * @return true if the StopPrimitives are not equal and false otherwise
     */
    bool operator!=(const StopPrimitive& other) const;

   private:
    unsigned int robot_id;

    // whether the robot should apply power to its wheels to stop
    // or should stop naturally, not applying power
    bool coast;
};
