#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "software/parameter/parameter.h"

template <class T>
class ContinousParameter : public Parameter<T>
{
    /**
     * ContinousParameters can only be templated with numeric types
     * we check using the is_arithmetic trait to disallow non-arithmetic values
     */
    static_assert(std::is_arithmetic<T>::value, "ContinousParameter: T must be numeric");

   public:
    /**
     * Constructs a new ContinousParameter, a Parameter that can take on any
     * value between a min and a max.
     *
     * NOTE: The templated type must be numeric (i.e int, double, float, etc..)
     *
     * @param name The name of the ContinousParameter
     * @param value The value for this ContinousParameter
     * @param min The minimum value this ContinousParameter can take
     * @param max The maximum value this ContinousParameter can take
     */
    explicit ContinousParameter<T>(const std::string& name, T value, T min, T max)
    {
        this->name_  = parameter_name;
        this->value_ = default_value;
        this->min_   = parameter_min;
        this->max_   = parameter_max;
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
     * Returns the min for this parameter
     *
     * @return the min of this parameter
     */
    const T getMin() const
    {
        return this->min_;
    }

    /**
     * Returns the max for this parameter
     *
     * @return the max of this parameter
     */
    const T getMax() const
    {
        return this->max_;
    }

    /**
     * Given the value, sets the value of this parameter and calls all registered
     * callback functions with the new value.
     *
     * @raises InvalidArgumentError if the new_value is not in allowed_values
     * @param new_value The new value to set
     */
    void setValue(const T new_value)
    {
        if (min_ <= new_value && new_value <= max_)
        {
            Parameter<T>::setValue(new_value);
        }
        else
        {
            LOG(WARNING) << "Rejected invalid parameter: " << parameter_name_;
            << "new value : " << new_value << " not in range ";
            << "Min: " << min_ << " and Max: " << max_ << std::endl;
        }
    }

   private:
    // store min and max value this param can be
    T min_;
    T max_;
};
