#pragma once

#include <memory>
#include <vector>
#include "util/parameter/parameter.h"

/**
 * This class defines a dynamic parameter of type integer, meaning the parameter
 * value can be changed during runtime
 */
class IntegerParameter : public Parameter
{
   public:
    /**
     * Constructs a new IntegerParameter
     * @param ros_parameter_path The global path in the ROS parameter server where this
     * parameter is stored
     * @param default_value The default value for this parameter
     */
    explicit IntegerParameter(const std::string& ros_parameter_path, int default_value);

    /**
     * Returns the value of this parameter
     *
     * @return the integer value of this parameter
     */
    const int value() const;

    void updateValueFromParameterServer() override;

    /**
     * Sets the parameter value in the ROS Parameter Server to the new value
     * @param new_value The new value to be set
     */
    void setValueInParameterServer(int new_value) const;

    /**
     * Returns a reference to the IntegerParameter registry. The registry is a list of
     * pointers to all the existing IntegerParameters.
     *
     * @return An immutable reference to the IntegerParameter registry
     */
    static const std::vector<std::shared_ptr<IntegerParameter>>& getRegistry();

    /**
     * Registers (adds) a IntegerParameter to the registry
     * @param integer_param A shared pointer to the IntegerParameter to add
     */
    static void registerParameter(std::shared_ptr<IntegerParameter> integer_param);

   private:
    /**
     * Returns a mutable reference to the IntegerParameter registry. This is the same as
     * the
     * above getRegistry() function, except that it returns a mutable reference. We need a
     * mutable version so that member functions can modify the registry. The function is
     * private so that external sources cannot modify the registry in unexpected ways.
     * @return A mutable reference to the IntegerParameter registry
     */
    static std::vector<std::shared_ptr<IntegerParameter>>& getMutableRegistry();

    // Store the value so it can be retrieved without fetching from the server again
    int value_;
};