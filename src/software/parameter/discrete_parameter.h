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
     * @param parameter_name The name of the parameter used by dynamic_reconfigure
     * dynamic_reconfigure
     * @param default_value The default value for this parameter
     */
    explicit Parameter<T>(const std::string& parameter_name, T default_value,
                          std::vector<T> parameter_options,
                          std::optional<T> parameter_min, std::optional<T> parameter_max)
    {
        this->name_    = parameter_name;
        this->value_   = default_value;
        this->min_     = parameter_min;
        this->max_     = parameter_max;
        this->options_ = parameter_options;
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
     * Returns the options for this parameter
     *
     * @return the options of this parameter
     */
    const std::vector<T> getOptions() const
    {
        return this->options_;
    }

    /**
     * Returns the min for this parameter
     *
     * @return the min of this parameter
     */
    const std::optional<T> getMin() const
    {
        return this->min_;
    }

    /**
     * Returns the max for this parameter
     *
     * @return the max of this parameter
     */
    const std::optional<T> getMax() const
    {
        return this->max_;
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
     * Registers a callback function to be called when the value of this parameter is
     * changed with setValue
     *
     * @param callback The function to call when this parameter's value is changed
     */
    void registerCallbackFunction(std::function<void(T)> callback)
    {
        std::scoped_lock callback_lock(this->callback_mutex_);
        callback_functions.emplace_back(callback);
    }

   private:
    // this mutex is marked as "mutable" so that it can be acquired in a const function
    mutable std::mutex value_mutex_;
    std::mutex callback_mutex_;

    // Store the value so it can be retrieved without fetching from the server again
    T value_;

    // Store the min/max
    // Its up to the user how they want to interpret min/max for types other than
    // int/double
    std::optional<T> min_;
    std::optional<T> max_;

    // Store the options of this parameter
    std::vector<T> options_;

    // Store the name of the parameter
    std::string name_;

    // A list of functions to call when a new parameter value is set
    std::vector<std::function<void(T)>> callback_functions;
};
