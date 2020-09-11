#pragma once
#include <any>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "software/util/typename/typename.h"

// A quality of life typedef to make things shorter and more readable
template <class IndexType, class TypeToCreate>
using ConstructorArgsGenericRegistry =
    std::unordered_map<IndexType, std::function<std::unique_ptr<TypeToCreate>(std::any)>>;
/**
 * The ConstructorArgsGenericFactory is an Abstract class that provides an interface for
 * Generic type Factories to follow. This makes it easy to maintain a list of factories
 * and get the corresponding generic types through the generic interface.
 *
 * Usage example:
 *
 * class Bar : public Foo (
 * public:
 *  Foo(int number, std::string name);
 * }
 *
 * To register Bar in the ConstructorArgsGenericFactory for Foo:
 *
 * static TConstructorArgsGenericFactory<std::string, Foo, Bar(int, std::string)> factory;
 *
 * First template parameter is the IndexType, second is the base class type, third is
 * the function signature of the constructor to register to the factory.
 *
 * To construct a `Bar` with apparent type `Foo` with the factory:
 * std::unique_ptr<Foo> bar_ptr =
 *      ConstructorArgsGenericFactory<std::string, Foo>::create("Bar", 5, "name");
 *
 * or
 *
 * std::unique_ptr<Foo> bar_ptr =
 *      ConstructorArgsGenericFactory<std::string, Foo>::create("Bar",
 * std::any(std::make_tuple(5, "name")));
 *
 */

// fn_traits is a template that allows us to access the return type and argument types
// of a function type or function.
template <typename Function>
class fn_traits;

// this is a template specialization of fn_traits for a function with return type
// ResultType and parameters ArgTypes...
template <typename ResultType, typename... ArgTypes>
class fn_traits<ResultType(ArgTypes...)>
{
   public:
    // the type of the tuple containing the types of arguments to the template parameter
    // function
    using args_tuple_type = std::tuple<ArgTypes...>;

    // the return type of the template parameter function
    using result_type = ResultType;
};


/**
 * The implementation for make_unique_from_tuple.
 * @tparam CreatedType The type to create a unique_ptr to with the given arguments
 * @tparam Tuple The type of the tuple containing the arguments to the constructor for
 * CreatedType
 * @tparam Idx an std::integer_sequence for [0, n) where n is the length of the tuple
 * @param tup The tuple containing the arguments to the constructor for CreatedType
 * @return a unique_ptr to an object of type CreatedType constructed with arguments in
 * `tup`
 */
template <typename CreatedType, typename Tuple, size_t... Idx>
std::unique_ptr<CreatedType> make_unique_from_tuple_impl(Tuple&& tup,
                                                         std::index_sequence<Idx...>)
{
    // std::get<Idx>(std::forward<Tuple>(tup))... expands into
    // std::get<1>(std::forward<Tuple>(tup)), std::get<2>(std::forward<Tuple>(tup)), ...
    // std::get<n>(std::forward<Tuple>(tup)) where n is the last number in the `Idx`
    // integer_sequence
    return std::unique_ptr<CreatedType>(
        new CreatedType(std::get<Idx>(std::forward<Tuple>(tup))...));
}

/**
 * A function that creates a unique_ptr to an object of CreatedType with constructor
 * arguments contained in the tuple `tup`.
 * @tparam CreatedType The type to create a unique_ptr to with the given arguments
 * @tparam Tuple The type of the tuple containing the arguments to the constructor for
 * CreatedType
 * @param tup The tuple containing the arguments to the constructor for CreatedType
 * @return a unique_ptr to an object of type CreatedType constructed with arguments in
 * `tup`
 */
template <typename CreatedType, typename Tuple>
std::unique_ptr<CreatedType> make_unique_from_tuple(Tuple&& tup)
{
    return make_unique_from_tuple_impl<CreatedType>(
        std::forward<Tuple>(tup),
        std::make_index_sequence<std::tuple_size_v<std::decay_t<Tuple>>>{});
}


template <class IndexType, class TypeToCreate>
class ConstructorArgsGenericFactory
{
   public:
    /**
     * Returns a unique pointer to a newly constructed type of the given type/name
     * constructed with the given arguments.
     *
     * @param generic_name The name of the type to construct. This value must be in the
     * Generic registry
     * @param constructor_args_tuple the tuple of constructor args, contained inside
     * std::any
     * @throws std::invalid_argument if the given generic_name is not found in the Generic
     * registry
     *
     * @return a unique pointer to a newly constructed type of the given type/name
     *
     * @throws std::invalid_argument if `constructor_args_tuple` cannot be cast to a tuple
     *         with the types in the constructor for `generic_name`
     */
    static std::unique_ptr<TypeToCreate> create(const IndexType& generic_name,
                                                std::any constructor_args_tuple);

    /**
     * Returns a unique pointer to a newly constructed type of the given type/name
     * constructed with the given arguments.
     * @tparam ArgTypes The types of the arguments to the type to construct
     * @param generic_name the name of the type to construct
     * @param args the arguments to construct the object of `generic_name` with
     * @return a unique pointer to a newly constructed type of the given type/name
     *
     * @throws std::invalid_argument if `args` are the wrong type for the constructor
     *         for `generic_name`
     */
    template <typename... ArgTypes>
    static std::unique_ptr<TypeToCreate> create(const IndexType& generic_name,
                                                ArgTypes... args);

    /**
     * Returns a const reference to the generic type registry. The registry is a map of
     * generic names to a "create" function that will create and return a unique_ptr to a
     * new concrete instance of the generic type.
     *
     * @return a const reference to the generic registry
     */
    static const ConstructorArgsGenericRegistry<IndexType, TypeToCreate>& getRegistry();

    /**
     * Returns the list of names for all creator functions registered in the factory
     *
     * @return a list of names for all creator functions registered in the factory
     */
    static std::vector<IndexType> getRegisteredNames();

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
     * @param generic_creator A "create" function that takes an std::any that  and will
     * return a unique_ptr to a new instance of the specified generic type
     */
    static void registerCreator(
        IndexType generic_name,
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
    static ConstructorArgsGenericRegistry<IndexType, TypeToCreate>& getMutableRegistry();
};

/**
 * This templated generic factory class is used by generic types that are derived from the
 * Abstract TypeToCreate template class. Its purpose is to create a Factory for the
 * implemented generic type and automatically register the Generic type in the
 * ConstructorArgsGenericFactory registry.
 *
 * Declaring the static variable will also cause it to be initialized at the start of the
 * program (because it's static). This will immediately call the constructor, which adds
 * the backend T to the ConstructorArgsGenericFactory registry. From then on, the rest of
 * the program can use the registry to find all the `TypeToCreate` that are available (and
 * register with this templated class).
 *
 * @tparam T The class of the generic type to be added to the registry. For example,
 * to add a new class called MoveBackend that inherits from Backend with a constructor
 * that accepts an `int` and an `std::string`, the following line should be added to the
 * end of the .cpp file (without the quotations):
 * "static TConstructorArgsGenericFactory<std::string, Backend,
 *                                        MoveBackend(int, std::string)> factory;"
 */
template <class IndexType, typename TypeToCreate, typename ConstructorType>
class TConstructorArgsGenericFactory
    : public ConstructorArgsGenericFactory<IndexType, TypeToCreate>
{
    using ConstructorFunctionTraits = fn_traits<ConstructorType>;

    // compile time type checking that T is derived class of Generic
    static_assert(
        std::is_base_of<TypeToCreate,
                        typename ConstructorFunctionTraits::result_type>::value,
        "ConstructorFunctionTraits::result_type must be derived class of TypeToCreate!");

    using args_tuple_type = typename ConstructorFunctionTraits::args_tuple_type;

   public:
    TConstructorArgsGenericFactory()
    {
        auto generic_creator = [](std::any arg) -> std::unique_ptr<TypeToCreate> {
            std::optional<args_tuple_type> constructor_args_or_null;

            try
            {
                // try to unpack the argument in `arg` to a tuple containing the types
                // accepted by a constructor of `ConstructorType`
                constructor_args_or_null = std::any_cast<args_tuple_type>(arg);
            }
            catch (const std::bad_any_cast& error)
            {
                std::stringstream error_ss;
                error_ss << "Argument type is wrong! Could not any_cast "
                         << demangle_typeid(arg.type().name())
                         << " to tuple of argument types " << TYPENAME(args_tuple_type);
                throw std::invalid_argument(error_ss.str());
            }

            return make_unique_from_tuple<
                typename ConstructorFunctionTraits::result_type>(
                *constructor_args_or_null);
        };
        ConstructorArgsGenericFactory<IndexType, TypeToCreate>::registerCreator(
            TYPENAME(typename ConstructorFunctionTraits::result_type), generic_creator);
    }
};
#include "software/util/design_patterns/constructor_args_generic_factory.tpp"
