#pragma once

#include "ai/primitive/primitive.h"

class StopPrimitive : public Primitive
{
    public:
     static const std::string PRIMITIVE_NAME;

    /**
     * Creates a new Stop Primitive
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param stop to stop the robot or not
     */
     explicit StopPrimitive(unsigned int robot_id, bool coast);

    /**
     * Creates a new Stop Primitive from a Primitive message
     *
     * @param primitive_msg The message from which to create the Stop Primitive
     */
     explicit StopPrimitive(const thunderbots_msgs::Primitive &primitive_msg);

     std::string getPrimitiveName() const override;

     unsigned int getRobotId() const override;

     bool coastRobot() const;

     std::vector<double> getParameters() const override;

     std::vector<bool> getExtraBits() const override;

    private:
     unsigned int robot_id;
     bool coast;

};