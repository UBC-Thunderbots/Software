#pragma once

#include <cxxabi.h>

#include <memory>

/**
 * Demangles typeid name
 *
 * @param mangled_name The mangled name to demangle
 *
 * @return the demangled string representation
 */
std::string demangleTypeId(const char* mangled_name);

/**
 * Gets the demangled typeid name of an object reference
 *
 * @param obj_ref The reference to the object
 *
 * @return the demangled string representation
 */
template <typename T>
std::string objectTypeName(const T& obj_ref)
{
return demangleTypeId(typeid(obj_ref).name());
}

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
#define CLASS_TYPENAME(object) (demangle_typeid(typeid(object).name()))
