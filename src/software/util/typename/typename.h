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
std::string demangle_typeid(const char* mangled_name);

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
#define TYPENAME(object) (demangle_typeid(typeid(object).name()))
