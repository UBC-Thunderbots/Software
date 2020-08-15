#include <cxxabi.h>

/**
 * MACRO to get typename of the object, i.e.
 * ClassA object_a;
 * TYPENAME(object_a); // returns ClassA
 *
 * @param object The object to get typename for
 */
#define TYPENAME(object)                                                                 \
    (static_cast<std::string>(abi::__cxa_demangle(typeid(object).name(), 0, 0, 0)))
