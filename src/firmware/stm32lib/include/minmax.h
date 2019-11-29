/**
 * \defgroup MINMAX Minimum and maximum macros
 * @{
 */

#ifndef STM32LIB_MINMAX_H
#define STM32LIB_MINMAX_H

/**
 * \brief Returns the smaller of two elements.
 *
 * \param a the first element
 *
 * \param b the second element
 *
 * \return the smaller element
 */
#define MIN(a, b) ((a) < (b) ? (a) : (b))

/**
 * \brief Returns the larger of two elements.
 *
 * \param a the first element
 *
 * \param b the second element
 *
 * \return the larger element
 */
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#endif

/**
 * @}
 */
