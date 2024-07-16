#pragma once

#include <type_traits>

/**
 * Type trait to check whether Derived inherits from any instantiation
 * of the class template Base.
 *
 * Taken from: https://stackoverflow.com/a/73034562
 *
 * TODO (#3242): Change is_template_base_of to use C++20 concepts
 *
 * @example See is_template_base_of_test.cpp
 *
 * @tparam Base the class template
 * @tparam Derived the type to check
 */
template <template <class...> class Base, class Derived, class = void>
struct is_template_base_of : std::false_type
{
};

template <template <class...> class Base, typename... Ts>
void test(Base<Ts...>&);

template <template <class...> class Base, class Derived>
struct is_template_base_of<Base, Derived,
                           std::void_t<decltype(test<Base>(std::declval<Derived&>()))>>
    : std::true_type
{
};
