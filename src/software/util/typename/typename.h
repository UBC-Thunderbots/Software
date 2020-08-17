#include <cxxabi.h>

#include <memory>

/**
 * MACRO to get typename of the object, i.e.
 * ```
 * ClassA object_a;
 * TYPENAME(object_a); // returns ClassA
 * ```
 *
 * @param object The object to get typename for
 *
 * @return the string representation of the object
 */
template <class T>
std::string TYPENAME(const T& object)
{
    std::unique_ptr<char, void (*)(void*)> demangled_name_ptr{
        abi::__cxa_demangle(typeid(object).name(), NULL, NULL, NULL), std::free};
    return std::string(demangled_name_ptr.get());
}
