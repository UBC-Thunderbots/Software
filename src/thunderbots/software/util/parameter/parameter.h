#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

/**
 * This class defines a dynamic parameter, meaning the parameter
 * value can be changed during runtime. Although the class is templated, it is only meant
 * to support the types also supported by the ROS Parameter Server.
 *
 * See http://wiki.ros.org/Parameter%20Server for the list of types
 *
 * In our codebase, we support bool, int32_t, double, string, and lists (aka vectors)
 * of the previous 4 types
 */
template <class T>
class Parameter
{
   public:
    /**
     * Constructs a new Parameter
     *
     * @param ros_parameter_path The global path in the ROS parameter server where this
     * parameter is stored
     * @param default_value The default value for this parameter
     */
    explicit Parameter<T>(const std::string& ros_parameter_path, T default_value)
    {
        this->ros_parameter_path = ros_parameter_path;
        value_                   = default_value;

        Parameter<T>::registerParameter(std::make_unique<Parameter<T>>(*this));
    }

    /**
     * Returns the global path in the ROS parameter server where this parameter is stored
     *
     * @return the global path in the ROS parameter sever where this parameter is stored
     */
    const std::string getROSParameterPath() const
    {
        return ros_parameter_path;
    }

    /**
     * Returns the value of this parameter
     *
     * @return the value of this parameter
     */
    const T value() const
    {
        return value_;
    }

    /**
     * Updates the value of this Parameter with the value from the ROS
     * Parameter Server
     */
    void updateValueFromROSParameterServer()
    {
        ros::param::get(getROSParameterPath(), value_);
    }

    /**
     * Sets the parameter value in the ROS Parameter Server to the new value.
     *
     * @param new_value The new value to be set
     */
    void setValueInROSParameterServer(T new_value)
    {
        ros::param::set(getROSParameterPath(), new_value);
    }

    /**
     * Returns a reference to the Parameter registry. The registry is a list of
     * pointers to all the existing Parameters.
     *
     * @return An immutable reference to the Parameter registry
     */
    static const std::vector<std::unique_ptr<Parameter<T>>>& getRegistry()
    {
        return Parameter<T>::getMutableRegistry();
    }

    /**
     * Registers (adds) a Parameter to the registry. Since the unique pointer is moved
     * into the registry, the pointer may not be accessed by the caller after this
     * function has been called.
     *
     * @param parameter A unique pointer to the Parameter to add. This pointer may not
     * be accessed by the caller after this function has been called.
     */
    static void registerParameter(std::unique_ptr<Parameter<T>> parameter)
    {
        Parameter<T>::getMutableRegistry().emplace_back(std::move(parameter));
    }

    /**
     * Updates all the Parameters of type T with the latest values from the ROS
     * Parameter Server
     */
    static void updateAllParametersFromROSParameterServer()
    {
        for (const auto& p : Parameter<T>::getRegistry())
        {
            p->updateValueFromROSParameterServer();
        }
    }

   private:
    /**
     * Returns a mutable reference to the Parameter registry. This is the same as the
     * above getRegistry() function, except that it returns a mutable reference. We need a
     * mutable version so that member functions can modify the registry. The function is
     * private so that external sources cannot modify the registry in unexpected ways.
     *
     * @return A mutable reference to the Parameter registry
     */
    static std::vector<std::unique_ptr<Parameter>>& getMutableRegistry()
    {
        static std::vector<std::unique_ptr<Parameter<T>>> instance;
        return instance;
    }

    // Store the value so it can be retrieved without fetching from the server again
    T value_;

    // The global path to the variable in the ROS Parameter Server
    std::string ros_parameter_path;
};
