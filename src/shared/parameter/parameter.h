#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

template <class T>
class Parameter
{
   public:
    /**
     * Constructs a new Parameter
     *
     * @param name The name of the parameter
     * @param value The value for this parameter
     */
    explicit Parameter<T>(const std::string& name, T value)
    {
        this->name_  = name;
        this->value_ = value;
    }

    /**
     * Returns the value of this parameter
     *
     * @return the value of this parameter
     */
    const T value() const
    {
        std::scoped_lock lock(this->value_mutex_);
        return this->value_;
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
     * Given the value, sets the value of this parameter and calls all registered
     * callback functions with the new value
     *
     * @param new_value The new value to set
     * @return true if the value has been set - in Parameter class, always true
     */
    virtual bool setValue(const T new_value)
    {
        std::scoped_lock value_lock(this->value_mutex_);
        this->value_ = new_value;

        std::scoped_lock callback_lock(this->callback_mutex_);
        for (auto callback_func : callback_functions)
        {
            callback_func(new_value);
        }

        return true;
    }

    /**
     * Registers a callback function to be called when the value of this parameter is
     * changed with setValue
     *
     * @param callback The function to call when this parameter's value is changed
     */
    void registerCallbackFunction(std::function<void(T)> callback) const
    {
        std::scoped_lock callback_lock(this->callback_mutex_);
        callback_functions.emplace_back(callback);
    }

   protected:
    // Store the value so it can be retrieved without fetching from the server again
    T value_;

    // Store the name of the parameter
    std::string name_;

    // A list of functions to call when a new parameter value is set
    // marked as mutable to allow const parameters to also register callbacks
    mutable std::vector<std::function<void(T)>> callback_functions;

   private:
    // mutexes are marked as mutable so that they can be acquired in a const function
    mutable std::recursive_mutex value_mutex_;
    mutable std::recursive_mutex callback_mutex_;
};
