#include "software/world/refbox_constants.h"

#include <magic_enum/magic_enum.hpp>

std::string name(const RefboxGameState& state)
{
    std::string find_name_result = static_cast<std::string>(magic_enum::enum_name(state));
    return find_name_result;
}

std::ostream& operator<<(std::ostream& os, const RefboxGameState& state)
{
    os << name(state);
    return os;
}
