#include "software/util/design_patterns/generic_factory.h"

// A quality of life typedef to make things shorter and more readable
template <class GenericType>
using GenericRegistry = std::unordered_map<std::string, std::function<std::unique_ptr<GenericType>()>>;

template <class GenericType>
GenericRegistry<GenericType>& GenericFactory<GenericType>::getMutableRegistry()
{
    static GenericRegistry<GenericType> instance;
    return instance;
}

template <class GenericType>
const GenericRegistry<GenericType>& GenericFactory<GenericType>::getRegistry()
{
    return GenericFactory::getMutableRegistry();
}

template <class GenericType>
std::vector<std::string> GenericFactory<GenericType>::getRegisteredGenericNames()
{
    std::vector<std::string> names;

    for (auto iter = GenericFactory<GenericType>::getRegistry().begin();
         iter != GenericFactory<GenericType>::getRegistry().end(); iter++)
    {
        names.emplace_back(iter->first);
    }
    return names;
}

template <class GenericType>
std::vector<std::function<std::unique_ptr<GenericType>()>>
GenericFactory<GenericType>::getRegisteredGenericConstructors()
{
    std::vector<std::function<std::unique_ptr<GenericType>()>> constructors;

    for (auto iter = GenericFactory::getRegistry().begin();
         iter != GenericFactory::getRegistry().end(); iter++)
    {
        constructors.emplace_back(iter->second);
    }
    return constructors;
}

template <class GenericType>
void GenericFactory<GenericType>::registerGeneric(
        std::string generic_name,
        std::function<std::unique_ptr<GenericType>()> generic_creator)
{
    GenericFactory::getMutableRegistry().insert(
            std::make_pair(generic_name, generic_creator));
}

template <class GenericType>
std::unique_ptr<GenericType> GenericFactory<GenericType>::createGeneric(const std::string& generic_name)
{
    auto registry = GenericFactory::getRegistry();
    auto iter     = registry.find(generic_name);
    if (iter != registry.end())
    {
        return iter->second();
    }
    else
    {
        std::string msg = std::string("No constructor for '" + generic_name +
                                      "' found in the GenericFactory");
        throw std::invalid_argument(msg);
    }
}
