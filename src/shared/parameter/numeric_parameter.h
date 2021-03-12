#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "shared/parameter/parameter.h"
#include "software/logger/logger.h"

template <class T>
class NumericParameter : public Parameter<T>
{
    /**
     * NumericParameters can only be templated with numeric types
     * we check using the is_arithmetic trait to disallow non-arithmetic types
     */
    static_assert(std::is_arithmetic<T>::value, "NumericParameter: T must be numeric");

   public:
    /**
     * Constructs a new NumericParameter, a Parameter that can take on any
     * value between a min and a max.
     *
     * NOTE: The templated type must be numeric (i.e int, double, float, etc..)
     *
     * A NumericParameter will never violate min <= value <= max
     *
     * @param name The name of the NumericParameter
     * @param value The value for this NumericParameter
     * @param min The minimum value this NumericParameter can take
     * @param max The maximum value this NumericParameter can take
     *
     * @raises std::invalid_argument if the value is already out of bounds
     */
    explicit NumericParameter<T>(const std::string& name, T value, T min, T max)
        : Parameter<T>(name, value)
    {
        if (value > max || min > value)
        {
            throw std::invalid_argument(
                "NumericParameter constructed with out of bounds value");
        }

        min_ = min;
        max_ = max;
    }

    /**
     * Returns the min for this parameter
     *
     * @return the min of this parameter
     */
    const T getMin() const
    {
        return min_;
    }

    /**
     * Returns the max for this parameter
     *
     * @return the max of this parameter
     */
    const T getMax() const
    {
        return max_;
    }

    /**
     * Given the value, checks if the value is between the min and max value, and if so
     * sets the value of this parameter and calls all registered callback functions with
     * the new value.
     *
     * If the new_value is not allowed, it will be rejected, and the old value is
     * preserved.
     *
     * @param new_value The new value to set
     * @returns true if the value was accepted, false if rejected
     */
    bool setValue(const T new_value) override
    {
        if (min_ <= new_value && new_value <= max_)
        {
            Parameter<T>::setValue(new_value);
            return true;
        }
        else
        {
            LOG(WARNING) << "Rejected out-of-range value " << new_value
                         << " to NumericParameter " << Parameter<T>::name_
                         << " while min: " << min_ << " and max: " << max_ << std::endl;
            return false;
        }
    }

   private:
    // store min and max value this param can be
    T min_;
    T max_;
};
