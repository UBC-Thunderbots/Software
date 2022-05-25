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
class EnumeratedParameter : public Parameter<T>
{
   public:
    /**
     * Constructs a new EnumeratedParameter, a Parameter that will only take
     * on a value from the allowed_values.
     *
     * A EnumeratedParameter will never hold a value outside of allowed_values
     *
     * @param name The name of the EnumeratedParameter
     * @param value The value for this EnumeratedParameter
     * @param allowed_values The values that this EnumeratedParameter is allowed to be
     *
     * @raises std::invalid_argument if the value is already invalid
     */
    explicit EnumeratedParameter<T>(const std::string& name, T value,
                                    std::vector<T> allowed_values)
        : Parameter<T>(name, value)
    {
        if (std::find(allowed_values.begin(), allowed_values.end(), value) ==
            allowed_values.end())
        {
            throw std::invalid_argument("EnumeratedParameter " + name +
                                        " constructed with invalid value");
        }

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
    bool setValue(const T new_value) override
    {
        if (std::find(allowed_values_.begin(), allowed_values_.end(), new_value) !=
            allowed_values_.end())
        {
            Parameter<T>::setValue(new_value);
            return true;
        }
        else
        {
            LOG(WARNING) << "Rejected new value " << new_value
                         << " to EnumeratedParameter " << Parameter<T>::name_
                         << std::endl;
            return false;
        }
    }

   private:
    // Store the options of this parameter
    std::vector<T> allowed_values_;
};
