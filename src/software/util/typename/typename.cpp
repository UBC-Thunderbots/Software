#include "software/util/typename/typename.h"

std::string demangleTypeId(const char* mangled_name)
{
    auto demangled_name_char_ptr = abi::__cxa_demangle(mangled_name, NULL, NULL, NULL);
    std::string demangled_name(demangled_name_char_ptr);
    free(demangled_name_char_ptr);
    return demangled_name;
}
