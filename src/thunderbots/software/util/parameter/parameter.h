#pragma once

#include <string>

/**
 * This Abstract class defines part of the interface for a Parameter object. Because of
 * the different return types for the different Parameters (bool vs int, etc.), there is
 * not an easy way of enforcing an interface for all the Parameter classes. The functions
 * that do not have Parameter-specific types are defined here.
 *
 * Otherwise, it is up to the developer to make sure that all subclasses of this Parameter
 * class provide the same interface.
 */
class Parameter
{
   public:
    /**
     * Returns the global path in the ROS parameter server where this parameter is stored
     * @return the global path in the ROS parameter sever where this parameter is stored
     */
    const std::string getROSParameterPath() const;

    /**
     * Updates the value of this BooleanParameter with the value from the ROS
     * Parameter Server
     */
    virtual void updateValueFromParameterServer() = 0;

    virtual ~Parameter() = default;

   protected:
    // The global path to the variable in the ROS Parameter Server
    std::string ros_parameter_path;
};
