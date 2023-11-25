#include <stdexcept>

#include "software/util/generic_factory/generic_factory.h"

template <class IndexType, class TypeToCreate, class ConfigType>
GenericRegistry<IndexType, TypeToCreate, ConfigType>&
GenericFactory<IndexType, TypeToCreate, ConfigType>::getMutableRegistry()
{
    static GenericRegistry<IndexType, TypeToCreate, ConfigType> instance;
    return instance;
}

template <class IndexType, class TypeToCreate, class ConfigType>
const GenericRegistry<IndexType, TypeToCreate, ConfigType>&
GenericFactory<IndexType, TypeToCreate, ConfigType>::getRegistry()
{
    return GenericFactory::getMutableRegistry();
}

template <class IndexType, class TypeToCreate, class ConfigType>
std::vector<std::string>
GenericFactory<IndexType, TypeToCreate, ConfigType>::getRegisteredNames()
{
    std::vector<std::string> names;
    auto registry = GenericFactory<IndexType, TypeToCreate, ConfigType>::getRegistry();

    for (auto iter = registry.begin(); iter != registry.end(); iter++)
    {
        names.emplace_back(iter->first);
    }
    return names;
}

template <class IndexType, class TypeToCreate, class ConfigType>
std::vector<std::function<std::unique_ptr<TypeToCreate>(const ConfigType)>>
GenericFactory<IndexType, TypeToCreate, ConfigType>::getRegisteredConstructors()
{
    std::vector<std::function<std::unique_ptr<TypeToCreate>(const ConfigType)>>
        constructors;
    auto registry = GenericFactory::getRegistry();

    for (auto iter = registry.begin(); iter != registry.end(); iter++)
    {
        constructors.emplace_back(iter->second);
    }
    return constructors;
}

template <class IndexType, class TypeToCreate, class ConfigType>
void GenericFactory<IndexType, TypeToCreate, ConfigType>::registerCreator(
    std::string generic_name,
    std::function<std::unique_ptr<TypeToCreate>(const ConfigType)> generic_creator)
{
    GenericFactory::getMutableRegistry().insert(
        std::make_pair(generic_name, generic_creator));
}

template <class IndexType, class TypeToCreate, class ConfigType>
std::unique_ptr<TypeToCreate> GenericFactory<IndexType, TypeToCreate, ConfigType>::create(
    const std::string& generic_name, const ConfigType config)
{
    auto registry = GenericFactory<IndexType, TypeToCreate, ConfigType>::getRegistry();
    auto iter     = registry.find(generic_name);
    if (iter != registry.end())
    {
        return iter->second(config);
    }
    else
    {
        std::string msg = std::string("No constructor for '" + generic_name +
                                      "' found in the GenericFactory");
        throw std::invalid_argument(msg);
    }
}
