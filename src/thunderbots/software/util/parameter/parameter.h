#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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
        // NOTE we can not use std::atmoic as we need to support strings
        if (Parameter<T>::getMutableRegistry().count(this->name_))
        {
            auto& param_in_registry = Parameter<T>::getMutableRegistry().at(this->name_);
            param_in_registry->param_lock_->lock();
            auto value = param_in_registry->value_;
            param_in_registry->param_lock_->unlock();

            return value;
        }
        // if the parameter hasn't been registered yet, return default value
        this->param_lock_->lock();
        auto value = this->value_;
        this->param_lock_->unlock();

        return value;
    }

    /*
     * Given the value, sets the value of this parameter
     *
     * @param new_value The new value to set
     */
    void setValue(const T new_value)
    {
        // get the value from the parameter in the registry
        // NOTE we can not use std::atmoic as we need to support strings
        if (Parameter<T>::getMutableRegistry().count(this->name_))
        {
            auto& param_in_registry = Parameter<T>::getMutableRegistry().at(this->name_);
            param_in_registry->param_lock_->lock();
            param_in_registry->value_ = new_value;
            param_in_registry->param_lock_->unlock();
        }

        // if the parameter hasn't been registered yet, set current value
        else
        {
            this->param_lock_->lock();
            this->value_ = new_value;
            this->param_lock_->unlock();
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

    // lock to read/write to param, we use a shared ptr to a mutex as the thread
    // needs to use the parameters lock to access the parameter in the registry.
    // If we don't use a pointer we cannot "std::move" a parameter into the registry.
    std::shared_ptr<std::mutex> param_lock_ = std::make_shared<std::mutex>();

    // Store the value so it can be retrieved without fetching from the server again
    T value_;

    // Store the name of the parameter
    std::string name_;

    // Store the namespace of the parameter
    std::string namespace_;
};
