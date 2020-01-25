#pragma once
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
//#include "software/backend/backend.h" // Should make generic.h

// A quality of life typedef to make things shorter and more readable
template <class IndexType, class TypeToCreate>
using GenericRegistry = std::unordered_map<IndexType, std::function<std::unique_ptr<TypeToCreate>()>>;
/**
 * The GenericFactory is an Abstract class that provides an interface for generic type
 * Factories to follow. This makes it easy to maintain a list of factories and get the
 * corresponding generic types through the generic interface.
 */
template <class IndexType, class TypeToCreate>
class GenericFactory
{
public:
    /**
     * Returns a unique pointer to a newly constructed Generic of the given type/name
     *
     * @param generic_name The name of the Generic to construct. This value must be in the
     * Generic registry
     * @throws std::invalid_argument if the given generic_name is not found in the Generic
     * registry
     *
     * @return a unique pointer to a newly constructed Generic of the given type/name
     */
    static std::unique_ptr<TypeToCreate> createGeneric(const std::string& generic_name);

    /**
     * Returns a const reference to the Generic registry. The registry is a map of Generic
     * names to a "create" function that will create and return a unique_ptr to a new
     * concrete instance of the Generic.
     *
     * @return a const reference to the Generic registry
     */
    static const GenericRegistry<IndexType, TypeToCreate>& getRegistry();

    /**
     * Returns a list of names of all the existing Generics
     *
     * @return a list of names of all the existing Generics
     */
    static std::vector<std::string> getRegisteredGenericNames();

    /**
     * Returns a list of constructor functions for all the existing Generics
     *
     * @return a list of constructor functions for all the existing Generics
     */
    static std::vector<std::function<std::unique_ptr<TypeToCreate>()>>
    getRegisteredGenericConstructors();

protected:
    /**
     * Adds a Generic to the Generic Registry
     *
     * @param generic_name The name of the Generic to be added
     * @param generic_creator A "create" function that takes no arguments and will return
     * a unique_ptr to a new instance of the specified Generic
     */
    static void registerGeneric(
            std::string generic_name,
            std::function<std::unique_ptr<TypeToCreate>()> generic_creator);

private:
    /**
     * Returns a reference to the Generic registry. The registry is a map of Generic names
     * to a "create" function that will create and return a unique_ptr to a new concrete
     * instance of the Generic, which allows the code to be aware
     * of all the Generics that are available.
     *
     * This is the same as the above public getRegistry function. We need a mutable
     * version in order to add entries to the registry. The function is private so that
     * only this class can make the modifications. Outside sources should not have direct
     * access to modify the registry.
     *
     * @return a mutable reference to the Generic registry
     */
    static GenericRegistry<IndexType, TypeToCreate>& getMutableRegistry();
};

/**
 * This templated generic factory class is used by Backends that are derived from the
 * Abstract Generic class. Its purpose is to create a Factory for the implemented Generic
 * and automatically register the backend in the BackendFactory registry.
 *
 * Declaring the static variable will also cause it to be initialized at the start of the
 * program (because it's static). This will immediately call the constructor, which adds
 * the backend T to the GenericFactory registry. From then on, the rest of the program
 * can use the registry to find all the Backends that are available (and register with
 * this templated class).
 *
 * @tparam T The class of the Generic to be added to the registry. For example, to add a
 * new class called MoveBackend that inherits from Backend, the following line should be
 * added to the end of the .cpp file (without the quotations): "static
 * TGenericFactory<MoveBackend> factory;"
 */
template <class IndexType, class TypeToCreate, class T>
class TGenericFactory : public GenericFactory<IndexType, TypeToCreate>
{
    // compile time type checking that T is derived class of Generic
    static_assert(std::is_base_of<TypeToCreate, T>::value,
                  "T must be derived class of TypeToCreate!");

public:
    TGenericFactory()
    {
        auto generic_creator = []() -> std::unique_ptr<TypeToCreate> {
            return std::make_unique<T>();
        };
        GenericFactory<IndexType, TypeToCreate>::registerGeneric(T::name, generic_creator);
    }
};
