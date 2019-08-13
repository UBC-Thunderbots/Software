#pragma once

#include <ros/ros.h>

#include <memory>
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
 * value can be changed during runtime.
 *
 * In our codebase, we currently support bool, int32_t, double, and strings
 *
 * */

template <class T>
class Parameter
{
   public:
    /**
     * Constructs a new Parameter
     *
     * @param parameter_name The name of the parameter used by dynamic_reconfigure
     * @param parameter_namespace The namespace of the parameter used to sort parameters
     * @param default_value The default value for this parameter
     */
    explicit Parameter<T>(const std::string& parameter_name,
                          const std::string& parameter_namespace, T default_value)
    {
        this->name_      = parameter_name;
        this->namespace_ = parameter_namespace;
        this->value_     = default_value;

        Parameter<T>::registerParameter(std::make_unique<Parameter<T>>(*this));
    }

    /**
     * Returns the value of this parameter
     *
     * @return the value of this parameter
     */
    const T value() const
    {
        // get the value from the parameter in the registry
        if (Parameter<T>::getMutableRegistry().count(this->name_))
        {
            auto& param_in_registry = Parameter<T>::getMutableRegistry().at(this->name_);
            // we load the atomic value using memory_order_relaxed as we do not
            // impose an order on memory accesses. This only garuntees atomicity and
            // modification order consistency. Ref:
            // https://en.cppreference.com/w/cpp/atomic/memory_order
            return param_in_registry->value_.load(std::memory_order_relaxed);
        }
        // if the parameter hasn't been registered yet, return default value
        return this->value_.load(std::memory_order_relaxed);
    }

    /*
     * Given the value, sets the value of this parameter
     *
     * @param new_value The new value to set
     */
    void setValue(const T new_value)
    {
        // get the value from the parameter in the registry
        if (Parameter<T>::getMutableRegistry().count(this->name_))
        {
            auto& param_in_registry = Parameter<T>::getMutableRegistry().at(this->name_);
            param_in_registry->value_.store(new_value, std::memory_order_relaxed);
        }

        // if the parameter hasn't been registered yet, set current value
        // we update the atomic value using memory_order_relaxed as we do not need to
        // impose an order on memory accesses. This only garuntees atomicity and
        // modification order consistency. Ref:
        // https://en.cppreference.com/w/cpp/atomic/memory_order
        else
        {
            this->value_.store(new_value, std::memory_order_relaxed);
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
     * Returns a reference to the Parameter registry. The registry is a list of
     * pointers to all the existing Parameters.
     *
     * @return An immutable reference to the Parameter registry
     */
    static const std::map<std::string, std::unique_ptr<Parameter<T>>>& getRegistry()
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
        Parameter<T>::getMutableRegistry().insert(
            std::pair<std::string, std::unique_ptr<Parameter<T>>>(parameter->name(),
                                                                  std::move(parameter)));
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
    static std::map<std::string, std::unique_ptr<Parameter<T>>>& getMutableRegistry()
    {
        static std::map<std::string, std::unique_ptr<Parameter<T>>> instance;
        return instance;
    }

    // Store the value so it can be retrieved without fetching from the server again
    std::atomic<T> value_;

    // Store the name of the parameter
    std::string name_;

    // Store the namespace of the parameter
    std::string namespace_;
};
