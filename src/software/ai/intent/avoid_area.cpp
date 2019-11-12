#include "software/ai/intent/avoid_area.h"

#include <magic_enum/magic_enum.hpp>
#include <string>

std::ostream& operator<<(std::ostream& os, const AvoidArea& area)
{
    std::string find_avoid_area_name_result =
        static_cast<std::string>(magic_enum::enum_name(area));
    os << find_avoid_area_name_result;

    return os;
}
