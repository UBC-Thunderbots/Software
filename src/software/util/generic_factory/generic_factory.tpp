#include <stdexcept>

#include "software/util/generic_factory/generic_factory.h"

template <class IndexType, class TypeToCreate, typename... Args>
GenericRegistry<IndexType, TypeToCreate, Args...>&
GenericFactory<IndexType, TypeToCreate, Args...>::getMutableRegistry()
{
    static GenericRegistry<IndexType, TypeToCreate, Args...> instance;
    return instance;
}

template <class IndexType, class TypeToCreate, typename... Args>
const GenericRegistry<IndexType, TypeToCreate, Args...>&
GenericFactory<IndexType, TypeToCreate, Args...>::getRegistry()
{
    return GenericFactory::getMutableRegistry();
}

template <class IndexType, class TypeToCreate, typename... Args>
std::vector<std::string>
GenericFactory<IndexType, TypeToCreate, Args...>::getRegisteredNames()
{
    std::vector<std::string> names;
    auto registry = GenericFactory<IndexType, TypeToCreate, Args...>::getRegistry();

    for (auto iter = registry.begin(); iter != registry.end(); iter++)
    {
        names.emplace_back(iter->first);
    }
    return names;
}

template <class IndexType, class TypeToCreate, typename... Args>
std::vector<std::function<std::unique_ptr<TypeToCreate>(Args...)>>
GenericFactory<IndexType, TypeToCreate, Args...>::getRegisteredConstructors()
{
    std::vector<std::function<std::unique_ptr<TypeToCreate>(Args...)>>
        constructors;
    auto registry = GenericFactory::getRegistry();

    for (auto iter = registry.begin(); iter != registry.end(); iter++)
    {
        constructors.emplace_back(iter->second);
    }
    return constructors;
}

template <class IndexType, class TypeToCreate, typename... Args>
void GenericFactory<IndexType, TypeToCreate, Args...>::registerCreator(
    std::string generic_name,
    std::function<std::unique_ptr<TypeToCreate>(Args...)> generic_creator)
{
    GenericFactory::getMutableRegistry().insert(
        std::make_pair(generic_name, generic_creator));
}

template <class IndexType, class TypeToCreate, typename... Args>
std::unique_ptr<TypeToCreate> GenericFactory<IndexType, TypeToCreate, Args...>::create(
    const std::string& generic_name, Args... args)
{
    auto registry = GenericFactory<IndexType, TypeToCreate, Args...>::getRegistry();
    auto iter     = registry.find(generic_name);
    if (iter != registry.end())
    {
        return iter->second(std::forward<Args>(args)...);
    }
    else
    {
        std::string msg = std::string("No constructor for '" + generic_name +
                                      "' found in the GenericFactory");
        throw std::invalid_argument(msg);
    }
}
