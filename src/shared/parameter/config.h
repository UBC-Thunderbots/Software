#pragma once

#include <string>
#include <variant>
#include <vector>

#include "shared/parameter/enumerated_parameter.h"
#include "shared/parameter/numeric_parameter.h"
#include "shared/parameter/parameter.h"

class Config;

// A MutableParameterVariant is a variant of shared_ptrs of parameters of
// int/bool/double/string or to a Config object which is a collection of parameters or
// configs itself. The objects in this variant can be mutated
using MutableParameterVariant = std::variant<
    std::shared_ptr<Parameter<bool>>, std::shared_ptr<Parameter<std::string>>,
    std::shared_ptr<NumericParameter<int>>, std::shared_ptr<NumericParameter<double>>,
    std::shared_ptr<EnumeratedParameter<std::string>>, std::shared_ptr<Config>>;

// A ParameterVariant is similar to a MutableParameterVariant, except the
// parameters/configs in ParameterVariant can not be mutated.
using ParameterVariant =
    std::variant<std::shared_ptr<const Parameter<bool>>,
                 std::shared_ptr<const Parameter<std::string>>,
                 std::shared_ptr<const NumericParameter<int>>,
                 std::shared_ptr<const NumericParameter<double>>,
                 std::shared_ptr<const EnumeratedParameter<std::string>>,
                 std::shared_ptr<const Config>>;

// List of ParameterVariants (immutable)
using ParameterList = std::vector<ParameterVariant>;

// List of MutableParameterVariants (mutable)
using MutableParameterList = std::vector<MutableParameterVariant>;

/* Config Class
 *
 * Pure virtual class used to represent a collection of
 * params and/or configs.
 */
class Config
{
   public:
    /* Pure virtual destructor */
    virtual ~Config() = 0;

    /*
     * Returns the name of the config
     *
     * returns std::string The name of the configuration
     */
    virtual const std::string name() const = 0;

    /*
     * Returns a const reference to the internal ParameterList
     *
     * returns const ParameterList& The parameter lists
     */
    virtual const ParameterList& getParameterList() const = 0;

    /*
     * Returns a reference to the internal ParameterList
     *
     * returns ParameterList& The parameter lists
     */
    virtual const MutableParameterList& getMutableParameterList() = 0;
};

inline Config::~Config() {}
