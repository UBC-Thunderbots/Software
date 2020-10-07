#include "software/util/typename/typename.h"

std::string demangle_typeid(const char* mangled_name)
{
    auto demangled_name_char_ptr = abi::__cxa_demangle(mangled_name, NULL, NULL, NULL);
    std::string demangled_name(demangled_name_char_ptr);
    delete demangled_name_char_ptr;
    return demangled_name;
}
