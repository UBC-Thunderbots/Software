#pragma once

#include <ros/ros.h>

#include <memory>
#include <mutex>
#include <string>

// messages for dynamic_reconfigure
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>

// message that contains arrays of the xmlrpc types for reconf
#include <dynamic_reconfigure/Config.h>

// message for the reconfigure srv, takes a config msg
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/config_tools.h>

/**
 * This class defines a dynamic parameter, meaning the parameter
 * value can be changed during runtime. Although the class is templated, it is only meant
 * to support the types also supported by the ROS Parameter Server.
 *
 * See http://wiki.ros.org/Parameter%20Server for the list of types
 *
 * In our codebase, we support bool, int32_t, double, and strings
 * */

template <class T>
class Parameter
{
   public:
    /**
     * Constructs a new Parameter
     *
     * @param parameter_name The name of the parameter used by dynamic_reconfigure
     * @param parameter_namespace The namespace of the parameter used by
     * dynamic_reconfigure
     * @param default_value The default value for this parameter
     */
    explicit Parameter<T>(const std::string& parameter_name,
                          const std::string& parameter_namespace, T default_value)
    {
        this->name_      = parameter_name;
        this->namespace_ = parameter_namespace;
        this->value_     = default_value;

        Parameter<T>::registerParameter(this);
    }

    /**
     * Returns the global path in the ROS parameter server where this parameter is stored
     *
     * @return the global path in the ROS parameter sever where this parameter is stored
     */
    const std::string getROSParameterPath() const
    {
        return "/" + this->namespace_ + "/" + name();
    }

    /**
     * Returns the value of this parameter
     *
     * @return the value of this parameter
     */
    T value()
    {
        std::scoped_lock lock(this->value_mutex_);
        return this->value_;
    }

    /**
     * Given the value, sets the value of this parameter
     *
     * @param new_value The new value to set
     */
    void setValue(const T new_value)
    {
        std::scoped_lock value_lock(this->value_mutex_);
        this->value_ = new_value;
        std::scoped_lock callback_lock(this->callback_mutex_);
        for (auto callback_func : callback_functions)
        {
            callback_func(new_value);
        }
    }

    /**
     * Returns the name of this parameter
     *
     * @return the name of this parameter
     */
    const std::string name() const
    {
        return name_;
    }

    /**
     * Checks if the parameter currently exists in the ros parameter server
     *
     * @return true if the parameter exists, false otherwise
     *
     */
    const bool existsInParameterServer() const
    {
        return ros::param::has(this->getROSParameterPath());
    }

    /**
     * Updates the value of this Parameter with the value from the ROS
     * Parameter Server
     */
    void updateValueFromROSParameterServer()
    {
        ros::param::get(getROSParameterPath(), this->value_);
    }

    /**
     * Updates the value of this Parameter with the value from a
     * 'dynamic_reconfigure::Config' msg. The parameter fetches the update from the update
     * msg and updates its value
     *
     */
    void updateParameterFromConfigMsg(
        const dynamic_reconfigure::Config::ConstPtr& updates)
    {
        dynamic_reconfigure::ConfigTools::getParameter(*updates, this->name_,
                                                       this->value_);
    }

    void registerCallbackFunction(std::function<void(T)> callback)
    {
        std::scoped_lock callback_lock(this->callback_mutex_);
        callback_functions.emplace_back(callback);
    }

    /**
     * Returns a reference to the Parameter registry. The registry is a list of
     * pointers to all the existing Parameters.
     *
     * @return An immutable reference to the Parameter registry
     */
    static const std::vector<Parameter<T>*>& getRegistry()
    {
        return Parameter<T>::getMutableRegistry();
    }

    /**
     * Returns a reference to the config msg. The config msg contains
     * all the current configurations
     *
     * @return An immutable reference to the Config msg
     */
    static const dynamic_reconfigure::Config& getConfigMsg()
    {
        return Parameter<T>::getMutableConfigMsg();
    }

    /**
     * Registers (adds) a Parameter to the registry. Since the unique pointer is moved
     * into the registry, the pointer may not be accessed by the caller after this
     * function has been called.
     *
     * Also registers params to the static configuration msg used to
     * set parameters
     *
     * @param parameter A unique pointer to the Parameter to add. This pointer may not
     * be accessed by the caller after this function has been called.
     */
    static void registerParameter(Parameter<T>* parameter)
    {
        try
        {
            dynamic_reconfigure::ConfigTools::appendParameter(
                Parameter<T>::getMutableConfigMsg(), parameter->name(),
                parameter->value());
        }
        catch (...)
        {
            // TODO (Issue #16): Replace with proper exception once exception handling is
            // implemented
            ROS_WARN("Attempting to configure with unkown type");
        }

        Parameter<T>::getMutableRegistry().emplace_back(parameter);
    }

    /**
     * Updates all the Parameters of type T with the latest values from the ROS
     * Parameter Server
     */
    static void updateAllParametersFromROSParameterServer()
    {
        for (const auto& param : Parameter<T>::getRegistry())
        {
            std::scoped_lock lock(param->value_mutex_);
            param->updateValueFromROSParameterServer();
        }
    }

    /**
     * Takes a list from the dynamic_reconfigure::Config msg and updates the parameters
     * based on the information in that list.
     *
     */
    static void updateAllParametersFromConfigMsg(
        const dynamic_reconfigure::Config::ConstPtr& updates)
    {
        for (const auto& param : Parameter<T>::getRegistry())
        {
            std::scoped_lock lock(param->value_mutex_);
            param->updateParameterFromConfigMsg(updates);
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
    static std::vector<Parameter<T>*>& getMutableRegistry()
    {
        // our registry needs to hold onto a unique mutex to access the parameters in the
        // registry as mutexes cannot be moved or copied
        static std::vector<Parameter<T>*> instance;
        return instance;
    }

    std::mutex value_mutex_;

    // Store the value so it can be retrieved without fetching from the server again
    T value_;

    // Store the name of the parameter
    std::string name_;

    // Store the namespace of the parameter
    std::string namespace_;

    std::mutex callback_mutex_;
    // A list of functions to call when a new parameter value is set
    std::vector<std::function<void(T)>> callback_functions;

    /**
     * Returns a mutable configuration msg that will hold all the
     * information related to the parameters created
     * msg contains bool,strs,ints,doubles vectors which are inherently mutable
     *
     * @return A mutable reference to the configuration msg
     */
    static dynamic_reconfigure::Config& getMutableConfigMsg()
    {
        static dynamic_reconfigure::Config config;
        return config;
    }
};
