#include "software/util/make_enum/make_enum.h"

std::vector<std::string> separateEnumStrings(std::string enum_string)
{
    enum_string.erase(remove_if(enum_string.begin(), enum_string.end(), isspace),
                      enum_string.end());
    std::vector<std::string> separated_enum_strings;
    std::string segment;
    std::stringstream enum_string_stream(enum_string);
    while (std::getline(enum_string_stream, segment, ','))
    {
        separated_enum_strings.emplace_back(segment);
    }

    return separated_enum_strings;
}
