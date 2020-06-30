#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "software/logger/logger.h"
#include "software/parameter/parameter.h"

template <class T>
class DiscreteParameter : public Parameter<T>
{
   public:
    /**
     * Constructs a new DiscreteParameter, a Parameter that only take
     * on a value from the allowed_values.
     *
     * @param name The name of the parameter
     * @param value The value for this parameter
     * @param allowed_values The values that this parameter is allowed to be
     */
    explicit DiscreteParameter<T>(const std::string& name, T value,
                                  std::vector<T> allowed_values)
        : Parameter<T>(name, value)
    {
        allowed_values_ = allowed_values;
    }

    /**
     * Returns the allowed values for this parameter
     *
     * @return the allowed values of this parameter
     */
    const std::vector<T> getAllowedValues() const
    {
        return allowed_values_;
    }

    /**
     * Given the value, checks if the value is in the allowed_values, and if so, sets the
     * value of this parameter and calls all registered callback functions with the new
     * value.
     *
     * If the new_value is not allowed, it will be rejected, and the old value is
     * preserved.
     *
     * @param new_value The new value to set
     * @returns true if the value was accepted, false if rejected
     */
    bool setValue(const T new_value)
    {
        if (std::find(allowed_values_.begin(), allowed_values_.end(), new_value) !=
            allowed_values_.end())
        {
            Parameter<T>::setValue(new_value);
            return true;
        }
        else
        {
            LOG(WARNING) << "Rejected new value " << new_value << " to DiscreteParameter "
                         << Parameter<T>::name_ << std::endl;
            return false;
        }
    }

   private:
    // Store the options of this parameter
    std::vector<T> allowed_values_;
};
