#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "software/parameter/parameter.h"

template <class T>
class DiscreteParameter : Parameter<T>
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
    explicit Parameter<T>(const std::string& name, T value, std::vector<T> allowed_values)
    {
        this->name_           = name;
        this->value_          = value;
        this->allowed_values_ = allowed_values;
    }

    /**
     * Returns the allowed values for this parameter
     *
     * @return the allowed values of this parameter
     */
    const std::vector<T> getAllowedValues() const
    {
        return this->allowed_values_;
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
        if (std::find(allowed_values_.begin(), allowed_values_.end(), new_value) {
            Parameter<T>::setValue(new_value);
        }
        else {
            LOG(WARNING) << "Rejected invalid parameter: " << parameter_name_;
            << " value not allowed" << new_value;
        }
    }

   private:
    // Store the options of this parameter
    std::vector<T> allowed_values_;
};
