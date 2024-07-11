#pragma once

#include <array>
#include <string_view>
#include <utility>

/**
 * Removes all leading and trailing whitespace from a string.
 *
 * @param str the string to strip whitespace from
 *
 * @return str stripped of all its leading and trailing whitespace
 */
constexpr std::string_view stripWhitespace(const std::string_view str)
{
    constexpr std::string_view WHITESPACE = " \t\n\f\r\v";

    const size_t str_begin = str.find_first_not_of(WHITESPACE);
    if (str_begin == std::string::npos)
    {
        return "";
    }

    const size_t str_end   = str.find_last_not_of(WHITESPACE);
    const size_t str_range = str_end - str_begin + 1;

    return str.substr(str_begin, str_range);
}

/**
 * Counts the number of substrings separated by a delimiter.
 *
 * @param str a string of substrings separated by the delimiter
 * @param delimiter the delimiting character between substrings
 *
 * @return the number of substrings in the string
 */
constexpr size_t countSubstrings(std::string_view str, char delimiter)
{
    if (str.empty())
    {
        return 0;
    }

    // There is always at least one substring if the string isn't empty
    size_t count = 1;
    for (size_t i = 0; i < str.size(); ++i)
    {
        // Trailing delimiters are not counted
        if (str.at(i) == delimiter && (i != str.size() - 1))
        {
            ++count;
        }
    }
    return count;
}

/**
 * Splits a string of substrings separated by a delimiter into an array containing
 * those substrings. All leading and trailing whitespace for each substring will be
 * removed.
 *
 * @param str a string of substrings separated by the delimiter
 * @param delimiter the delimiting character between substrings
 * @param index_seq an index sequence 0, 1, 2, ..., (N - 1) where N is the number of
 * substrings in str (use countSubstrings to count the number of substrings)
 *
 * @return an array containing the substrings
 */
template <size_t... INDEX_SEQ>
constexpr auto splitString(std::string_view str, [[maybe_unused]] char delimiter,
                           std::index_sequence<INDEX_SEQ...> index_seq)
{
    std::array<std::string_view, sizeof...(INDEX_SEQ)> result = {};
    std::string_view remaining                                = str;
    size_t delimiter_pos                                      = 0;

    // This pattern will be instantiated for each index in index_seq
    ((delimiter_pos = remaining.find(delimiter),

      // Get the substring before the first delimiter in the remaining string
      result[INDEX_SEQ] = stripWhitespace(remaining.substr(0, delimiter_pos)),

      // Set remaining string to everything after the first delimiter
      remaining = (delimiter_pos == std::string_view::npos)
                      ? ""
                      : remaining.substr(delimiter_pos + 1)),
     ...);

    return result;
}

/**
 * Checks if a string representing enum values is valid.
 *
 * An enum string is valid if it does not contain any '=', meaning
 * the enum is not being assigned any values manually.
 *
 * @param str a string containing comma-separated enum value names
 *
 * @return true if the string is valid, and false otherwise
 */
constexpr bool isEnumArgsValid(const std::string_view& str)
{
    return str.find('=') == std::string_view::npos;
}

/**
 * Returns an array containing the values of the enum.
 *
 * @tparam ENUM the enum
 * @tparam NUM_VALUES the number of values in the enum
 *
 * @return an array containing the values of the enum
 */
template <typename ENUM, size_t NUM_VALUES>
constexpr auto getEnumValues()
{
    std::array<ENUM, NUM_VALUES> values = {};
    for (size_t i = 0; i < NUM_VALUES; ++i)
    {
        // This casting relies on the assumption that the enum does not manually
        // specify any values, because if it did we may cast to different values
        // or miss some values entirely when constructing the vector
        values[i] = static_cast<ENUM>(i);
    }
    return values;
}
