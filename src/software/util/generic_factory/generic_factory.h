#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "software/util/typename/typename.h"

// A quality of life typedef to make things shorter and more readable
template <class IndexType, class TypeToCreate, class ConfigType>
using GenericRegistry =
    std::unordered_map<IndexType,
                       std::function<std::unique_ptr<TypeToCreate>(const ConfigType)>>;
/**
 * The GenericFactory is an Abstract class that provides an interface for Generic type
 * Factories to follow. This makes it easy to maintain a list of factories and get the
 * corresponding generic types through the generic interface.
 */
template <class IndexType, class TypeToCreate, class ConfigType>
class GenericFactory
{
   public:
    /**
     * Returns a unique pointer to a newly constructed type of the given type/name
     *
     * @param generic_name The name of the type to construct. This value must be in the
     * Generic registry
     * @throws std::invalid_argument if the given generic_name is not found in the Generic
     * registry
     *
     * @return a unique pointer to a newly constructed type of the given type/name
     */
    static std::unique_ptr<TypeToCreate> create(const std::string& generic_name,
                                                ConfigType config);

    /**
     * Returns a const reference to the generic type registry. The registry is a map of
     * generic names to a "create" function that will create and return a unique_ptr to a
     * new concrete instance of the generic type.
     *
     * @return a const reference to the generic registry
     */
    static const GenericRegistry<IndexType, TypeToCreate, ConfigType>& getRegistry();

    /**
     * Returns the list of names for all creator functions registered in the factory
     *
     * @return a list of names for all creator functions registered in the factory
     */
    static std::vector<std::string> getRegisteredNames();

    /**
     * Returns the list of creator functions that are registered in this factory
     *
     * @return a list of creator functions that are registered in this factory
     */
    static std::vector<std::function<std::unique_ptr<TypeToCreate>(const ConfigType)>>
    getRegisteredConstructors();

   protected:
    /**
     * Adds a generic creator function to the registry
     *
     * @param generic_name The name of the generic creator function to be added
     * @param generic_creator A "create" function that takes no arguments and will return
     * a unique_ptr to a new instance of the specified generic type
     */
    static void registerCreator(
        std::string generic_name,
        std::function<std::unique_ptr<TypeToCreate>(const ConfigType)> generic_creator);

   private:
    /**
     * Returns a reference to the generic registry. The registry is a map of Generic names
     * to a "create" function that will create and return a unique_ptr to a new concrete
     * instance of the generic type, which allows the code to be aware
     * of all the Generics that are available.
     *
     * This is the same as the above public getRegistry function. We need a mutable
     * version in order to add entries to the registry. The function is private so that
     * only this class can make the modifications. Outside sources should not have direct
     * access to modify the registry.
     *
     * @return a mutable reference to the generic registry
     */
    static GenericRegistry<IndexType, TypeToCreate, ConfigType>& getMutableRegistry();
};

/**
 * This templated generic factory class is used by generic types that are derived from the
 * Abstract TypeToCreate template class. Its purpose is to create a Factory for the
 * implemented generic type and automatically register the Generic type in the
 * GenericFactory registry.
 *
 * Declaring the static variable will also cause it to be initialized at the start of the
 * program (because it's static). This will immediately call the constructor, which adds
 * the backend T to the GenericFactory registry. From then on, the rest of the program
 * can use the registry to find all the Backends that are available (and register with
 * this templated class).
 *
 * @tparam T The class of the generic type neric to be added to the registry. For example,
 * to add a new class called MoveBackend that inherits from Backend (or any other
 * Generic), the following line should be added to the end of the .cpp file (without the
 * quotations): "static TGenericFactory<MoveBackend> factory;"
 */
template <class IndexType, class TypeToCreate, class T, class ConfigType>
class TGenericFactory : public GenericFactory<IndexType, TypeToCreate, ConfigType>
{
    // compile time type checking that T is derived class of Generic
    static_assert(std::is_base_of<TypeToCreate, T>::value,
                  "T must be derived class of TypeToCreate!");

   public:
    TGenericFactory()
    {
        // TODO (Issue #1142): Change to use a function instead of a static variable
        auto generic_creator =
            [](const ConfigType config) -> std::unique_ptr<TypeToCreate> {
            return std::make_unique<T>(config);
        };
        GenericFactory<IndexType, TypeToCreate, ConfigType>::registerCreator(
            TYPENAME(T), generic_creator);
    }
};
#include "software/util/generic_factory/generic_factory.tpp"
