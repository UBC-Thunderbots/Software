#pragma once

#include <memory>
#include <vector>
#include "util/parameter/parameter.h"

/**
 * This class defines a dynamic parameter of type boolean, meaning the parameter
 * value can be changed during runtime
 */
class BooleanParameter : public Parameter
{
   public:
    /**
     * Constructs a new BooleanParameter
     * @param ros_parameter_path The global path in the ROS parameter server where this
     * parameter is stored
     * @param default_value The default value for this parameter
     */
    explicit BooleanParameter(const std::string& ros_parameter_path, bool default_value);

    /**
     * Returns the value of this parameter
     *
     * @return the boolean value of this parameter
     */
    const bool value() const;

    void updateValueFromParameterServer() override;

    /**
     * Sets the parameter value in the ROS Parameter Server to the new value
     * @param new_value The new value to be set
     */
    void setValueInParameterServer(bool new_value) const;

    /**
     * Returns a reference to the BooleanParameter registry. The registry is a list of
     * pointers to all the existing BooleanParameters.
     *
     * @return An immutable reference to the BooleanParameter registry
     */
    static const std::vector<std::shared_ptr<BooleanParameter>>& getRegistry();

    /**
     * Registers (adds) a BooleanParameter to the registry
     * @param bool_param A shared pointer to the BooleanParameter to add
     */
    static void registerParameter(std::shared_ptr<BooleanParameter> bool_param);

   private:
    /**
     * Returns a mutable reference to the BooleanParameter registry. This is the same as
     * the
     * above getRegistry() function, except that it returns a mutable reference. We need a
     * mutable version so that member functions can modify the registry. The function is
     * private so that external sources cannot modify the registry in unexpected ways.
     * @return A mutable reference to the BooleanParameter registry
     */
    static std::vector<std::shared_ptr<BooleanParameter>>& getMutableRegistry();

    // Store the value so it can be retrieved without fetching from the server again
    bool value_;
};