/**
 * \defgroup FORMAT Hexadecimal formatting functions
 * @{
 */

#include <format.h>

/**
 * \brief Formats a 4-bit nybble as a single hexadecimal digit.
 *
 * \param dest the location at which to store the digit
 *
 * \param val the nybble to convert
 */
void formathex4(char *dest, uint8_t val)
{
    static const char DIGITS[16] = "0123456789ABCDEF";
    *dest                        = DIGITS[val & 0x0F];
}

/**
 * \brief Formats an 8-bit byte as two hexadecimal digts.
 *
 * \param dest the location at which to store the digits
 *
 * \param val the byte to convert
 */
void formathex8(char *dest, uint8_t val)
{
    formathex4(dest, val >> 4);
    formathex4(dest + 1, val);
}

/**
 * \brief Formats a 16-bit word as four hexadecimal digits.
 *
 * \param dest the location at which to store the digits
 *
 * \param val the word to convert
 */
void formathex16(char *dest, uint16_t val)
{
    formathex8(dest, val >> 8);
    formathex8(dest + 2, val);
}

/**
 * \brief Formats a 32-bit word as eight hexadecimal digits.
 *
 * \param dest the location at which to store the digits
 *
 * \param val the word to convert
 */
void formathex32(char *dest, uint32_t val)
{
    formathex16(dest, val >> 16);
    formathex16(dest + 4, val);
}

/**
 * @}
 */
