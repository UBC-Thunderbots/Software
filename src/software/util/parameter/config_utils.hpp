#pragma once

#include <string>
#include <variant>
#include <vector>

#include "software/util/parameter/parameter.h"

class Config;

// A ParameterVariant is a variant of shared_ptrs of parameters of int/bool/double/string
// or to a Config object which is a collection of parameters or configs iteslf.
using MutableParameterVariant =
    std::variant<std::shared_ptr<Parameter<int>>, std::shared_ptr<Parameter<bool>>,
                 std::shared_ptr<Parameter<double>>,
                 std::shared_ptr<Parameter<std::string>>, std::shared_ptr<Config>>;

using ParameterVariant = std::variant<
    std::shared_ptr<const Parameter<int>>, std::shared_ptr<const Parameter<bool>>,
    std::shared_ptr<const Parameter<double>>,
    std::shared_ptr<const Parameter<std::string>>, std::shared_ptr<const Config>>;

using ParameterList        = std::vector<ParameterVariant>;
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
