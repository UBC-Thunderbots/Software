#include "software/util/design_patterns/constructor_args_generic_factory.h"

template <class IndexType, class TypeToCreate>
ConstructorArgsGenericRegistry<IndexType, TypeToCreate>&
ConstructorArgsGenericFactory<IndexType, TypeToCreate>::getMutableRegistry()
{
    static ConstructorArgsGenericRegistry<IndexType, TypeToCreate> instance;
    return instance;
}

template <class IndexType, class TypeToCreate>
const ConstructorArgsGenericRegistry<IndexType, TypeToCreate>&
ConstructorArgsGenericFactory<IndexType, TypeToCreate>::getRegistry()
{
    return ConstructorArgsGenericFactory::getMutableRegistry();
}

template <class IndexType, class TypeToCreate>
std::vector<IndexType>
ConstructorArgsGenericFactory<IndexType, TypeToCreate>::getRegisteredNames()
{
    std::vector<IndexType> names;

    for (const auto& [name, creator_fn] : getRegistry())
    {
        // shut up the compiler
        (void)creator_fn;
        names.emplace_back(name);
    }
    return names;
}

template <class IndexType, class TypeToCreate>
std::vector<std::function<std::unique_ptr<TypeToCreate>(std::any)>>
ConstructorArgsGenericFactory<IndexType, TypeToCreate>::getRegisteredConstructors()
{
    std::vector<std::function<std::unique_ptr<TypeToCreate>(std::any)>> constructors;

    for (const auto& [name, creator_fn] : getRegistry())
    {
        // shut up the compiler
        (void)name;
        constructors.emplace_back(creator_fn);
    }
    return constructors;
}

template <class IndexType, class TypeToCreate>
void ConstructorArgsGenericFactory<IndexType, TypeToCreate>::registerCreator(
    IndexType generic_name,
    std::function<std::unique_ptr<TypeToCreate>(std::any)> generic_creator)
{
    ConstructorArgsGenericFactory::getMutableRegistry().insert(
        std::make_pair(generic_name, generic_creator));
}

template <class IndexType, class TypeToCreate>
std::unique_ptr<TypeToCreate>
ConstructorArgsGenericFactory<IndexType, TypeToCreate>::create(
    const IndexType& generic_name, std::any constructor_args_tuple)
{
    auto registry = ConstructorArgsGenericFactory<IndexType, TypeToCreate>::getRegistry();
    auto iter     = registry.find(generic_name);
    if (iter != registry.end())
    {
        return iter->second(constructor_args_tuple);
    }
    else
    {
        std::string msg = std::string("No constructor for '" + generic_name +
                                      "' found in the ConstructorArgsGenericFactory");
        throw std::invalid_argument(msg);
    }
}


template <class IndexType, class TypeToCreate>
template <typename... ArgTypes>
std::unique_ptr<TypeToCreate>
ConstructorArgsGenericFactory<IndexType, TypeToCreate>::create(
    const IndexType& generic_name, ArgTypes... args)
{
    auto registry = ConstructorArgsGenericFactory<IndexType, TypeToCreate>::getRegistry();
    auto iter     = registry.find(generic_name);
    if (iter != registry.end())
    {
        return iter->second(std::any(std::tuple<ArgTypes...>(args...)));
    }
    else
    {
        std::string msg = std::string("No constructor for '" + generic_name +
                                      "' found in the ConstructorArgsGenericFactory");
        throw std::invalid_argument(msg);
    }
}
