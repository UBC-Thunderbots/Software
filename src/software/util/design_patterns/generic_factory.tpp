#include "software/util/design_patterns/generic_factory.h"

template <class IndexType, class TypeToCreate>
GenericRegistry<IndexType, TypeToCreate>&
GenericFactory<IndexType, TypeToCreate>::getMutableRegistry()
{
    static GenericRegistry<IndexType, TypeToCreate> instance;
    return instance;
}

template <class IndexType, class TypeToCreate>
const GenericRegistry<IndexType, TypeToCreate>&
GenericFactory<IndexType, TypeToCreate>::getRegistry()
{
    return GenericFactory::getMutableRegistry();
}

template <class IndexType, class TypeToCreate>
std::vector<std::string> GenericFactory<IndexType, TypeToCreate>::getRegisteredNames()
{
    std::vector<std::string> names;

    for (auto iter = GenericFactory<IndexType, TypeToCreate>::getRegistry().begin();
         iter != GenericFactory<IndexType, TypeToCreate>::getRegistry().end(); iter++)
    {
        names.emplace_back(iter->first);
    }
    return names;
}

template <class IndexType, class TypeToCreate>
std::vector<std::function<std::unique_ptr<TypeToCreate>()>>
GenericFactory<IndexType, TypeToCreate>::getRegisteredConstructors()
{
    std::vector<std::function<std::unique_ptr<TypeToCreate>()>> constructors;

    for (auto iter = GenericFactory::getRegistry().begin();
         iter != GenericFactory::getRegistry().end(); iter++)
    {
        constructors.emplace_back(iter->second);
    }
    return constructors;
}

template <class IndexType, class TypeToCreate>
void GenericFactory<IndexType, TypeToCreate>::registerCreator(
    std::string generic_name,
    std::function<std::unique_ptr<TypeToCreate>()> generic_creator)
{
    GenericFactory::getMutableRegistry().insert(
        std::make_pair(generic_name, generic_creator));
}

template <class IndexType, class TypeToCreate, class ConfigType>
std::unique_ptr<TypeToCreate> GenericFactory<IndexType, TypeToCreate, ConfigType>::create(
    std::shared_ptr<const ConfigType> config, const std::string& generic_name)
{
    auto registry = GenericFactory<IndexType, TypeToCreate>::getRegistry();
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
