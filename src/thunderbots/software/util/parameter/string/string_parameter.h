#pragma once

#include <memory>
#include <vector>
#include "util/parameter/parameter.h"

/**
 * This class defines a dynamic parameter of type string, meaning the parameter
 * value can be changed during runtime
 */
class StringParameter : public Parameter
{
   public:
    /**
     * Constructs a new StringParameter
     * @param ros_parameter_path The global path in the ROS parameter server where this
     * parameter is stored
     * @param default_value The default value for this parameter
     */
    explicit StringParameter(const std::string& ros_parameter_path,
                             std::string default_value);

    /**
     * Returns the value of this parameter
     *
     * @return the string value of this parameter
     */
    const std::string value() const;

    void updateValueFromParameterServer() override;

    /**
     * Sets the parameter value in the ROS Parameter Server to the new value
     * @param new_value The new value to be set
     */
    void setValueInParameterServer(std::string new_value) const;

    /**
     * Returns a reference to the StringParameter registry. The registry is a list of
     * pointers to all the existing StringParameters.
     *
     * @return An immutable reference to the StringParameter registry
     */
    static const std::vector<std::shared_ptr<StringParameter>>& getRegistry();

    /**
     * Registers (adds) a StringParameter to the registry
     * @param string_param A shared pointer to the StringParameter to add
     */
    static void registerParameter(std::shared_ptr<StringParameter> string_param);

   private:
    /**
     * Returns a mutable reference to the StringParameter registry. This is the same as
     * the
     * above getRegistry() function, except that it returns a mutable reference. We need a
     * mutable version so that member functions can modify the registry. The function is
     * private so that external sources cannot modify the registry in unexpected ways.
     * @return A mutable reference to the StringParameter registry
     */
    static std::vector<std::shared_ptr<StringParameter>>& getMutableRegistry();

    // Store the value so it can be retrieved without fetching from the server again
    std::string value_;
};