#pragma once

#include <memory>
#include <vector>
#include "util/parameter/parameter.h"

/**
 * This class defines a dynamic parameter of type double, meaning the parameter
 * value can be changed during runtime
 */
class DoubleParameter : public Parameter
{
   public:
    /**
     * Constructs a new DoubleParameter
     * @param ros_parameter_path The global path in the ROS parameter server where this
     * parameter is stored
     * @param default_value The default value for this parameter
     */
    explicit DoubleParameter(const std::string& ros_parameter_path, double default_value);

    /**
     * Returns the value of this parameter
     *
     * @return the double value of this parameter
     */
    const double value() const;

    void updateValueFromParameterServer() override;

    /**
     * Sets the parameter value in the ROS Parameter Server to the new value
     * @param new_value The new value to be set
     */
    void setValueInParameterServer(double new_value) const;

    /**
     * Returns a reference to the DoubleParameter registry. The registry is a list of
     * pointers to all the existing DoubleParameters.
     *
     * @return An immutable reference to the DoubleParameter registry
     */
    static const std::vector<std::shared_ptr<DoubleParameter>>& getRegistry();

    /**
     * Registers (adds) a DoubleParameter to the registry
     * @param double_param A shared pointer to the DoubleParameter to add
     */
    static void registerParameter(std::shared_ptr<DoubleParameter> double_param);

   private:
    /**
     * Returns a mutable reference to the DoubleParameter registry. This is the same as
     * the
     * above getRegistry() function, except that it returns a mutable reference. We need a
     * mutable version so that member functions can modify the registry. The function is
     * private so that external sources cannot modify the registry in unexpected ways.
     * @return A mutable reference to the DoubleParameter registry
     */
    static std::vector<std::shared_ptr<DoubleParameter>>& getMutableRegistry();

    // Store the value so it can be retrieved without fetching from the server again
    double value_;
};