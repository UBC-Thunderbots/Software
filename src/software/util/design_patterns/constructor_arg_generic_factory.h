#pragma once
#include <any>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <unordered_map>
#include <vector>
//#include "software/backend/backend.h" // Should make generic.h

// A quality of life typedef to make things shorter and more readable
template <class IndexType, class TypeToCreate>
using ConstructorArgGenericRegistry =
std::unordered_map<IndexType, std::function<std::unique_ptr<TypeToCreate>(std::any)>>;
/**
 * The GenericFactory is an Abstract class that provides an interface for Generic type
 * Factories to follow. This makes it easy to maintain a list of factories and get the
 * corresponding generic types through the generic interface.
 */
template <class IndexType, class TypeToCreate>
class ConstructorArgGenericFactory
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
    template <typename U>
    static std::unique_ptr<TypeToCreate> create(const std::string& generic_name, U constructor_arg);

    static std::unique_ptr<TypeToCreate> create(const std::string& generic_name, std::any constructor_arg);

    /**
     * Returns a const reference to the generic type registry. The registry is a map of
     * generic names to a "create" function that will create and return a unique_ptr to a
     * new concrete instance of the generic type.
     *
     * @return a const reference to the generic registry
     */
    static const ConstructorArgGenericRegistry<IndexType, TypeToCreate>& getRegistry();

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
    static std::vector<std::function<std::unique_ptr<TypeToCreate>(std::any)>>
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
        std::function<std::unique_ptr<TypeToCreate>(std::any)> generic_creator);

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
    static ConstructorArgGenericRegistry<IndexType, TypeToCreate>& getMutableRegistry();
};


template <class IndexType, class TypeToCreate, class T, class ArgType>
class TConstructorArgGenericFactory : public ConstructorArgGenericFactory<IndexType, TypeToCreate>
{
    // compile time type checking that T is derived class of Generic
    static_assert(std::is_base_of<TypeToCreate, T>::value,
                  "T must be derived class of TypeToCreate!");

   public:
    TConstructorArgGenericFactory()
    {
        auto generic_creator = [](std::any arg) -> std::unique_ptr<TypeToCreate> {
            std::optional<ArgType> constructor_arg_or_null;

            try {
                constructor_arg_or_null = std::any_cast<ArgType>(arg);
            } catch (const std::bad_any_cast& error) {
                std::stringstream error_ss;
                error_ss << "Argument type is wrong! Could not any_cast "
                         << arg.type().name() << " to ArgType " << typeid(T).name();
                throw std::invalid_argument(error_ss.str());
            }

            return std::make_unique<T>(*constructor_arg_or_null);
        };
        ConstructorArgGenericFactory<IndexType, TypeToCreate>::registerCreator(T::name,
                                                                 generic_creator);
    }
};
#include "software/util/design_patterns/constructor_arg_generic_factory.tpp"
