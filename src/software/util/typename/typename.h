#include <cxxabi.h>

#define TYPENAME(object)                                                                 \
    (static_cast<std::string>(abi::__cxa_demangle(typeid(object).name(), 0, 0, 0)))
