#include "software/util/design_patterns/constructor_arg_generic_factory.h"

template <class IndexType, class TypeToCreate>
ConstructorArgGenericRegistry<IndexType, TypeToCreate>&
ConstructorArgGenericFactory<IndexType, TypeToCreate>::getMutableRegistry()
{
    static ConstructorArgGenericRegistry<IndexType, TypeToCreate> instance;
    return instance;
}

template <class IndexType, class TypeToCreate>
const ConstructorArgGenericRegistry<IndexType, TypeToCreate>&
ConstructorArgGenericFactory<IndexType, TypeToCreate>::getRegistry()
{
    return ConstructorArgGenericFactory::getMutableRegistry();
}

template <class IndexType, class TypeToCreate>
std::vector<std::string> ConstructorArgGenericFactory<IndexType, TypeToCreate>::getRegisteredNames()
{
    std::vector<std::string> names;

    for (auto iter = ConstructorArgGenericFactory<IndexType, TypeToCreate>::getRegistry().begin();
         iter != ConstructorArgGenericFactory<IndexType, TypeToCreate>::getRegistry().end(); iter++)
    {
        names.emplace_back(iter->first);
    }
    return names;
}

template <class IndexType, class TypeToCreate>
std::vector<std::function<std::unique_ptr<TypeToCreate>(std::any)>>
ConstructorArgGenericFactory<IndexType, TypeToCreate>::getRegisteredConstructors()
{
    std::vector<std::function<std::unique_ptr<TypeToCreate>(std::any)>> constructors;

    for (auto iter = ConstructorArgGenericFactory::getRegistry().begin();
         iter != ConstructorArgGenericFactory::getRegistry().end(); iter++)
    {
        constructors.emplace_back(iter->second);
    }
    return constructors;
}

template <class IndexType, class TypeToCreate>
void ConstructorArgGenericFactory<IndexType, TypeToCreate>::registerCreator(
    std::string generic_name,
    std::function<std::unique_ptr<TypeToCreate>(std::any)> generic_creator)
{
    ConstructorArgGenericFactory::getMutableRegistry().insert(
        std::make_pair(generic_name, generic_creator));
}

template <class IndexType, class TypeToCreate>
template <class U>
std::unique_ptr<TypeToCreate> ConstructorArgGenericFactory<IndexType, TypeToCreate>::create(
    const std::string& generic_name, U constructor_arg)
{
    auto registry = ConstructorArgGenericFactory<IndexType, TypeToCreate>::getRegistry();
    auto iter     = registry.find(generic_name);
    if (iter != registry.end())
    {
        return iter->second(std::any(constructor_arg));
    }
    else
    {
        std::string msg = std::string("No constructor for '" + generic_name +
                                      "' found in the ConstructorArgGenericFactory");
        throw std::invalid_argument(msg);
    }
}
