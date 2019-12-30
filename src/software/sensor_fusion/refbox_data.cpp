#include "software/sensor_fusion/refbox_data.h"

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

std::string name(const RefboxStage& stage)
{
    std::string find_name_result = static_cast<std::string>(magic_enum::enum_name(stage));
    return find_name_result;
}

std::ostream& operator<<(std::ostream& os, const RefboxStage& stage)
{
    os << name(stage);
    return os;
}
