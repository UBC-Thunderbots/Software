#include "software/util/typename/typename.h"

std::string demangle_typeid(const char* mangled_name)
{
    std::unique_ptr<char, void (*)(void*)> demangled_name_ptr{
        abi::__cxa_demangle(mangled_name, NULL, NULL, NULL), std::free};
    return std::string(demangled_name_ptr.get());
}
